// Joint-space teleoperation of one dVRK PSM from a Geomagic Touch; PhysX does the FK.
// From DVRK_META, Assets/Script/BACKUP.cs (first PSM_Controller, later replaced by 03).
// Omitted: HapticManager and ECM camera classes from the same file.
using UnityEngine;

public class PSM_Controller : MonoBehaviour
{
    public enum DeviceSide { Left, Right }

    [Header("0. Device")]
    public DeviceSide side = DeviceSide.Left;
    private int myDeviceId = -1;

    [Header("1. Arm joints (RCM yaw/pitch + insertion)")]
    public ArticulationBody Joint_L1_Yaw;
    public ArticulationBody Joint_L2_Pitch;
    public ArticulationBody Joint_L3_Insertion;
    // Parallelogram links that follow the pitch joint to keep the RCM fixed.
    public SlaveData[] slaves;

    [Header("2. Wrist and jaws")]
    public ArticulationBody Joint_Wrist_Roll;
    public ArticulationBody Joint_Jaw1;
    public ArticulationBody Joint_Jaw2;

    [Header("3. Passive joints (locked)")]
    public ArticulationBody[] passiveJoints;

    [Header("4. Master-to-slave scale factors")]
    public float scaleRot = 0.8f;          // stylus mm -> deg (yaw/pitch)
    public float scaleInsert = -0.0015f;   // stylus mm -> insertion m
    public float scaleWrist = 1.0f;

    [Header("5. Jaw settings")]
    public float jawOpenAngle = 30.0f;
    public float jawClosedAngle = 0.0f;
    public float jawSpeed = 100.0f;
    public bool invertJaw2 = true;

    [Header("6. Anti-jitter filter")]
    public float smoothSpeed = 5.0f;
    public float deadZone = 1f;

    private float[] rawPos = new float[3];
    private float[] rawRot = new float[3];
    private Vector3 smoothedPos;
    private float smoothedRotRoll;

    private bool isClutched = false;
    private float currentJawTarget = 0f;
    private float currentJawValue = 0f;
    private Vector3 handStart;
    private float handRotStart;
    private float startL1, startL2, startL3, startWrist;

    void Start()
    {
        SetupActiveJoint(Joint_L1_Yaw);
        SetupActiveJoint(Joint_L2_Pitch);
        SetupActiveJoint(Joint_L3_Insertion);
        SetupActiveJoint(Joint_Wrist_Roll);
        SetupActiveJoint(Joint_Jaw1);
        SetupActiveJoint(Joint_Jaw2);
        foreach (var s in slaves) SetupActiveJoint(s.joint);
        foreach (var p in passiveJoints) LockJoint(p);

        currentJawTarget = jawClosedAngle;
        currentJawValue = jawClosedAngle;
    }

    void FixedUpdate()
    {
        if (HapticManager.Instance == null) return;
        myDeviceId = (side == DeviceSide.Left) ? HapticManager.Instance.idLeft : HapticManager.Instance.idRight;
        if (myDeviceId < 0) return;

        HapticManager.GetPosition(myDeviceId, rawPos);
        HapticManager.GetRotation(myDeviceId, rawRot);
        int buttons = HapticManager.GetButtons(myDeviceId);

        Vector3 targetPos = new Vector3(rawPos[0], rawPos[1], rawPos[2]);
        float targetRotRoll = rawRot[2]; // gimbal roll [rad]

        // Exponential smoothing; inside the dead zone pass through so the
        // filter does not add micro-oscillations.
        if (Vector3.Distance(smoothedPos, targetPos) > deadZone)
            smoothedPos = Vector3.Lerp(smoothedPos, targetPos, Time.fixedDeltaTime * smoothSpeed);
        else
            smoothedPos = targetPos;

        smoothedRotRoll = Mathf.Lerp(smoothedRotRoll, targetRotRoll, Time.fixedDeltaTime * smoothSpeed);

        // Grey button opens the jaws, white closes.
        bool btnGrey = (buttons & 1) != 0;
        bool btnWhite = (buttons & 2) != 0;

        if (btnGrey) currentJawTarget = jawOpenAngle;
        else if (btnWhite) currentJawTarget = jawClosedAngle;

        currentJawValue = Mathf.MoveTowards(currentJawValue, currentJawTarget, Time.fixedDeltaTime * jawSpeed);

        if (Joint_Jaw1 && Joint_Jaw2)
        {
            SetDrive(Joint_Jaw1, currentJawValue);
            SetDrive(Joint_Jaw2, invertJaw2 ? -currentJawValue : currentJawValue);
        }

        // Coupled only while the clutch is held. Mapping is relative to the pose
        // at engage, so there is no jump and the hand can be re-centred.
        bool clutchInput = Input.GetKey(KeyCode.Space);

        if (clutchInput && !isClutched)
        {
            handStart = smoothedPos;
            handRotStart = smoothedRotRoll;

            if (Joint_L1_Yaw) startL1 = Joint_L1_Yaw.xDrive.target;
            if (Joint_L2_Pitch) startL2 = Joint_L2_Pitch.xDrive.target;
            if (Joint_L3_Insertion) startL3 = Joint_L3_Insertion.xDrive.target;
            if (Joint_Wrist_Roll) startWrist = Joint_Wrist_Roll.xDrive.target;

            isClutched = true;
        }
        else if (!clutchInput && isClutched)
        {
            isClutched = false;
        }

        // x -> yaw, y -> pitch, z -> insertion, stylus roll -> wrist roll.
        if (isClutched)
        {
            float dX = smoothedPos.x - handStart.x;
            float dY = smoothedPos.y - handStart.y;
            float dZ = smoothedPos.z - handStart.z;
            float dRot = (smoothedRotRoll - handRotStart) * Mathf.Rad2Deg;

            if (Joint_L1_Yaw) SetDrive(Joint_L1_Yaw, startL1 + (dX * scaleRot));

            if (Joint_L2_Pitch)
            {
                float targetL2 = startL2 - (dY * scaleRot);
                SetDrive(Joint_L2_Pitch, targetL2);
                foreach (var s in slaves) SetDrive(s.joint, s.invert ? -targetL2 : targetL2);
            }

            if (Joint_L3_Insertion) SetDrive(Joint_L3_Insertion, startL3 + (dZ * scaleInsert));
            if (Joint_Wrist_Roll) SetDrive(Joint_Wrist_Roll, startWrist + (dRot * scaleWrist));
        }
    }

    // Degrees for revolute joints, metres for prismatic.
    void SetDrive(ArticulationBody ab, float val)
    {
        if (ab == null) return;
        var drive = ab.xDrive;
        drive.target = val;
        ab.xDrive = drive;
    }

    void SetupActiveJoint(ArticulationBody ab)
    {
        if (ab == null) return;
        var drive = ab.xDrive;
        drive.stiffness = 10000f;
        drive.damping = 5000f;
        drive.forceLimit = 10000f;
        ab.xDrive = drive;
    }

    // Very stiff drive at zero freezes the joints that are not teleoperated.
    void LockJoint(ArticulationBody ab)
    {
        if (ab == null) return;
        var drive = ab.xDrive;
        drive.stiffness = 200000f;
        drive.damping = 10000f;
        drive.target = 0f;
        ab.xDrive = drive;
    }

    [System.Serializable]
    public struct SlaveData { public ArticulationBody joint; public bool invert; }
}
