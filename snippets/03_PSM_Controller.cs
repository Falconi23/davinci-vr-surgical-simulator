// Per-arm PSM control loop: Cartesian target -> one IK step -> joint drives, plus jaws and force feedback.
// From DVRK_META, Assets/Script/PSM_controller.cs. Omitted: unused fields left over from 01.
// Public field names are kept as-is because Unity serializes them in the scene.
using UnityEngine;

public class PSM_Controller : MonoBehaviour
{
    public enum DeviceSide { Left, Right }

    [Header("0. Device")]
    public DeviceSide side = DeviceSide.Left;
    private int myDeviceId = -1;

    [Header("1. Arm joints")]
    public ArticulationBody Joint_L1_Yaw;
    public ArticulationBody Joint_L2_Pitch;
    public ArticulationBody Joint_L3_Insertion;
    public SlaveData[] slaves; // parallelogram links mirror the pitch to keep the RCM fixed

    [Header("2. Wrist and jaws")]
    public ArticulationBody Joint_Wrist_Roll;
    public ArticulationBody Joint_Wrist_Pitch;
    public ArticulationBody Joint_Wrist_Yaw;
    public ArticulationBody Joint_Jaw1;
    public ArticulationBody Joint_Jaw2;

    [Header("3. Passive joints (locked)")]
    public ArticulationBody[] passiveJoints;

    [Header("4. Scale factors")]
    public float scaleRot = 0.8f;          // stylus -> target displacement
    public float scaleInsert = -0.0015f;
    public float scaleWristRoll = 1.0f;
    public float scaleWristPitch = 1.0f;
    public float scaleWristYaw = 1.0f;

    [Header("5. Jaw settings")]
    public float jawOpenAngle = 30.0f;
    public float jawClosedAngle = 0.0f;
    public float jawSpeed = 100.0f;
    public bool invertJaw2 = true;

    [Header("6. Anti-jitter filter")]
    public float smoothSpeed = 5.0f;
    public float deadZone = 1f;

    [Header("7. Keyboard clutch")]
    public KeyCode tastoFrizione = KeyCode.Space;   // clutch

    [Header("8. Force feedback")]
    public float forceScale = 1.0f;
    public HapticContactSensor tcpSensor;

    [Header("9. Inverse kinematics")]
    public Transform targetEE;
    public JacobianIKSolver ikSolver;

    [Header("10. Keyboard debug")]
    public bool usaTastieraDebug = false;           // drive the target from the keyboard, to test IK alone
    public float velocitaTastiera = 0.005f;         // [m/step]
    public float velocitaRotazioneTastiera = 1.5f;  // [deg/step]

    private float[] forceToSend = new float[3];
    private float[] rawPos = new float[3];
    private float[] rawRot = new float[3];

    private bool isClutched = false;
    private float currentJawTarget = 0f;
    private float currentJawValue = 0f;
    private Vector3 handStart;

    void Start()
    {
        SetupActiveJoint(Joint_L1_Yaw);
        SetupActiveJoint(Joint_L2_Pitch);
        SetupActiveJoint(Joint_L3_Insertion);
        SetupActiveJoint(Joint_Wrist_Roll);
        SetupActiveJoint(Joint_Wrist_Pitch);
        SetupActiveJoint(Joint_Wrist_Yaw);
        SetupActiveJoint(Joint_Jaw1);
        SetupActiveJoint(Joint_Jaw2);

        foreach (var s in slaves) SetupActiveJoint(s.joint);
        foreach (var p in passiveJoints) LockJoint(p);

        // Avoid a jump on start: drives start at the current joint reading and
        // the IK target at the actual tool tip.
        SyncDriveWithCurrentAngle(Joint_L1_Yaw);
        SyncDriveWithCurrentAngle(Joint_L2_Pitch);
        SyncDriveWithCurrentAngle(Joint_L3_Insertion);
        SyncDriveWithCurrentAngle(Joint_Wrist_Roll);
        SyncDriveWithCurrentAngle(Joint_Wrist_Pitch);

        if (targetEE != null && ikSolver != null && ikSolver.endEffector != null)
        {
            targetEE.position = ikSolver.endEffector.position;
            targetEE.rotation = ikSolver.endEffector.rotation;
        }

        currentJawTarget = jawClosedAngle;
        currentJawValue = jawClosedAngle;
    }

    void FixedUpdate()
    {
        if (usaTastieraDebug && targetEE != null)
        {
            float moveX = 0f, moveY = 0f, moveZ = 0f;

            if (Input.GetKey(KeyCode.D)) moveX = velocitaTastiera;
            if (Input.GetKey(KeyCode.A)) moveX = -velocitaTastiera;
            if (Input.GetKey(KeyCode.W)) moveZ = velocitaTastiera;
            if (Input.GetKey(KeyCode.S)) moveZ = -velocitaTastiera;
            if (Input.GetKey(KeyCode.Q)) moveY = velocitaTastiera;
            if (Input.GetKey(KeyCode.E)) moveY = -velocitaTastiera;

            targetEE.Translate(new Vector3(moveX, moveY, moveZ), Space.World);

            float rotX = 0f, rotY = 0f, rotZ = 0f;

            if (Input.GetKey(KeyCode.I)) rotX = velocitaRotazioneTastiera;
            if (Input.GetKey(KeyCode.K)) rotX = -velocitaRotazioneTastiera;
            if (Input.GetKey(KeyCode.J)) rotY = -velocitaRotazioneTastiera;
            if (Input.GetKey(KeyCode.L)) rotY = velocitaRotazioneTastiera;
            if (Input.GetKey(KeyCode.U)) rotZ = velocitaRotazioneTastiera;
            if (Input.GetKey(KeyCode.O)) rotZ = -velocitaRotazioneTastiera;

            // Self space, so rotations are relative to the current gripper orientation.
            targetEE.Rotate(new Vector3(rotX, rotY, rotZ), Space.Self);
        }

        // One IK iteration per physics step.
        if (ikSolver != null && targetEE != null)
        {
            float[] newJointTargets = ikSolver.CalculateIK();

            if (newJointTargets.Length >= 6)
            {
                SetDrive(Joint_L1_Yaw, newJointTargets[0]);

                SetDrive(Joint_L2_Pitch, newJointTargets[1]);
                foreach (var s in slaves)
                    SetDrive(s.joint, s.invert ? -newJointTargets[1] : newJointTargets[1]);

                SetDrive(Joint_L3_Insertion, newJointTargets[2]);
                SetDrive(Joint_Wrist_Roll, newJointTargets[3]);
                SetDrive(Joint_Wrist_Pitch, newJointTargets[4]);
                SetDrive(Joint_Wrist_Yaw, newJointTargets[5]);
            }
        }

        if (HapticManager.Instance == null) return;

        myDeviceId = (side == DeviceSide.Left) ? HapticManager.Instance.idLeft : HapticManager.Instance.idRight;
        if (myDeviceId < 0) return;

        HapticManager.GetPosition(myDeviceId, rawPos);
        HapticManager.GetRotation(myDeviceId, rawRot);
        int buttons = HapticManager.GetButtons(myDeviceId);

        Vector3 handCurrentPos = new Vector3(rawPos[0], rawPos[1], rawPos[2]);

        // Incremental while clutched, so the hand can be re-centred with the
        // clutch released without moving the instrument.
        if (!usaTastieraDebug)
        {
            bool clutchInput = Input.GetKey(tastoFrizione);

            if (clutchInput && !isClutched)
            {
                handStart = handCurrentPos;
                isClutched = true;
            }
            else if (!clutchInput && isClutched)
            {
                isClutched = false;
            }

            if (isClutched && targetEE != null)
            {
                Vector3 deltaHand = (handCurrentPos - handStart) * scaleRot;
                targetEE.position += deltaHand;
                handStart = handCurrentPos;
            }
        }

        bool btnGrey = (buttons & 1) != 0;   // open
        bool btnWhite = (buttons & 2) != 0;  // close

        if (btnGrey) currentJawTarget = jawOpenAngle;
        else if (btnWhite) currentJawTarget = jawClosedAngle;

        currentJawValue = Mathf.MoveTowards(currentJawValue, currentJawTarget, Time.fixedDeltaTime * jawSpeed);

        if (Joint_Jaw1 && Joint_Jaw2)
        {
            SetDrive(Joint_Jaw1, currentJawValue);
            SetDrive(Joint_Jaw2, invertJaw2 ? -currentJawValue : currentJawValue);
        }

        // Zero force when not coupled, so the stylus moves freely.
        if (isClutched && !usaTastieraDebug)
        {
            Vector3 force = ComputeHapticForce() * forceScale;
            forceToSend[0] = force.x;
            forceToSend[1] = force.y;
            forceToSend[2] = force.z;
        }
        else
        {
            forceToSend[0] = 0f; forceToSend[1] = 0f; forceToSend[2] = 0f;
        }
        HapticManager.SetForce(myDeviceId, forceToSend);
    }

    private Vector3 ComputeHapticForce()
    {
        if (tcpSensor != null) return tcpSensor.GetHapticForce();
        return Vector3.zero;
    }

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

    // Very stiff drive at zero freezes the joints that are not controlled.
    void LockJoint(ArticulationBody ab)
    {
        if (ab == null) return;
        var drive = ab.xDrive;
        drive.stiffness = 200000f;
        drive.damping = 10000f;
        drive.target = 0f;
        ab.xDrive = drive;
    }

    // jointPosition is in radians for revolute joints but drive targets are in degrees.
    void SyncDriveWithCurrentAngle(ArticulationBody ab)
    {
        if (ab == null) return;

        var drive = ab.xDrive;
        if (ab.jointType == ArticulationJointType.PrismaticJoint)
            drive.target = ab.jointPosition[0];
        else
            drive.target = ab.jointPosition[0] * Mathf.Rad2Deg;
        ab.xDrive = drive;
    }

    [System.Serializable]
    public struct SlaveData { public ArticulationBody joint; public bool invert; }
}
