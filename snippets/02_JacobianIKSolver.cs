// Jacobian-transpose IK for the dVRK PSM: no matrix inversion, so it stays
// well-behaved near singularities. From DVRK_META, Assets/Script/JacobianIKSolver.cs.
using UnityEngine;

public class JacobianIKSolver : MonoBehaviour
{
    [Header("Kinematic references")]
    public Transform endEffector;
    public Transform target;

    [Header("Solver settings")]
    public float learningRate = 0.5f;
    public float distanceThreshold = 0.001f; // [m]

    [Header("Orientation (vector alignment)")]
    public bool useOrientation = true;
    public float angleThreshold = 0.1f;
    public float orientationWeight = 2.0f;

    [Header("Joint configuration")]
    public ArticulationBody[] joints;
    public bool[] isPrismatic;
    public Vector3[] rotationAxes;           // local frame
    public bool[] affectsOrientation;

    [Header("Per-joint weights (redundancy handling)")]
    public float[] jointWeights = new float[] { 1f, 1f, 1f, 1f, 1f, 1f };

    [Header("Prismatic safety")]
    public float prismaticScale = 0.01f;
    public float maxPrismaticStep = 0.02f;   // per iteration [m]

    private float[] currentAngles;

    void Start()
    {
        currentAngles = new float[joints.Length];

        // Yaw, pitch and insertion position the tip; the wrist joints orient it.
        if (affectsOrientation == null || affectsOrientation.Length != joints.Length)
        {
            affectsOrientation = new bool[joints.Length];
            for (int i = 0; i < joints.Length; i++)
                affectsOrientation[i] = (i >= 3);
        }
    }

    /// <summary>One solver iteration; returns the new joint targets.</summary>
    public float[] CalculateIK()
    {
        Vector3 positionError = target.position - endEffector.position;

        // Cross products of the z and y axes instead of Euler angles: points
        // along the corrective axis and grows with misalignment, no gimbal lock.
        Vector3 angularError = Vector3.zero;
        float angleDist = 0f;

        if (useOrientation)
        {
            Vector3 errorZ = Vector3.Cross(endEffector.forward, target.forward); // shaft direction
            Vector3 errorY = Vector3.Cross(endEffector.up, target.up);           // roll about the shaft
            angularError = errorZ + errorY;
            angleDist = angularError.magnitude;
        }

        if (positionError.magnitude < distanceThreshold && (!useOrientation || angleDist < angleThreshold))
            return GetCurrentTargets();

        float[] deltaQ = new float[joints.Length];

        // Project the error onto each Jacobian column.
        for (int i = 0; i < joints.Length; i++)
        {
            Vector3 worldAxis = joints[i].transform.TransformDirection(rotationAxes[i]).normalized;
            Vector3 pivotToEE = endEffector.position - joints[i].transform.position;

            if (useOrientation && affectsOrientation[i])
            {
                if (jointWeights[i] <= 0f)
                {
                    deltaQ[i] = 0f;
                    continue;
                }

                float angularContribution = Vector3.Dot(worldAxis, angularError);
                deltaQ[i] = angularContribution * orientationWeight * jointWeights[i];
            }
            else
            {
                if (isPrismatic[i])
                {
                    // Lower gain within 5 cm to avoid overshooting in depth.
                    float linearContribution = Vector3.Dot(worldAxis, positionError);
                    if (positionError.magnitude < 0.05f)
                        deltaQ[i] = linearContribution * jointWeights[i] * 0.1f;
                    else
                        deltaQ[i] = linearContribution * jointWeights[i];
                }
                else
                {
                    // z_i x (p_ee - p_i)
                    Vector3 jLinear = Vector3.Cross(worldAxis, pivotToEE);
                    float linearContribution = Vector3.Dot(jLinear, positionError);
                    deltaQ[i] = linearContribution * jointWeights[i];
                }
            }
        }

        for (int i = 0; i < joints.Length; i++)
        {
            float delta = deltaQ[i] * learningRate;

            if (isPrismatic[i])
            {
                delta *= prismaticScale;
                delta = Mathf.Clamp(delta, -maxPrismaticStep, maxPrismaticStep);
            }
            else if (!affectsOrientation[i])
            {
                delta = Mathf.Clamp(delta, -2f, 2f); // deg per iteration
            }

            currentAngles[i] = joints[i].xDrive.target + delta;
            currentAngles[i] = Mathf.Clamp(currentAngles[i], joints[i].xDrive.lowerLimit, joints[i].xDrive.upperLimit);
        }

        return currentAngles;
    }

    private float[] GetCurrentTargets()
    {
        for (int i = 0; i < joints.Length; i++) currentAngles[i] = joints[i].xDrive.target;
        return currentAngles;
    }
}
