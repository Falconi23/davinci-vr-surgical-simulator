// Tool-tip contact sensor: penalty force F = k * depth * n from Physics.ComputePenetration.
// From DVRK_META, Assets/Script/HapticContactSensor.cs (logs translated; public field names
// kept because Unity serializes them).
using UnityEngine;

// Organs are capsule colliders tagged "Tissue": mesh colliders made the force
// feedback unstable, capsules give fast, robust penetration queries.
public class HapticContactSensor : MonoBehaviour
{
    [Header("Sensor settings")]
    public string tagTessuto = "Tissue";
    public float rigiditaMuro = 1000.0f;      // contact stiffness k

    [Header("Debugging")]
    public bool mostraLog = true;

    private Vector3 forceToApply = Vector3.zero;
    private Collider mioCollider;             // trigger

    void Start()
    {
        mioCollider = GetComponent<Collider>();
    }

    // Runs before the trigger callbacks of the same step, so the force is
    // non-zero only while in contact.
    void FixedUpdate()
    {
        forceToApply = Vector3.zero;
    }

    void OnTriggerStay(Collider other)
    {
        if (!other.CompareTag(tagTessuto)) return;

        if (mostraLog) Debug.Log("<color=yellow>Sensor trigger detected.</color>");

        Vector3 direction;
        float distance;

        bool isPenetrating = Physics.ComputePenetration(
            mioCollider, transform.position, transform.rotation,
            other, other.transform.position, other.transform.rotation,
            out direction, out distance
        );

        if (isPenetrating)
        {
            forceToApply = direction * (distance * rigiditaMuro);

            if (mostraLog)
                Debug.Log($"<color=lime>Force computed.</color> Depth: {distance:F4} | Force: {forceToApply.magnitude:F2}");
        }
    }

    public Vector3 GetHapticForce()
    {
        return forceToApply;
    }
}
