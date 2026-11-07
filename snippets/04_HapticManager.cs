// Singleton owning both Geomagic Touch devices through the native bridge in 06_Dll_DaVinci.cpp.
// From DVRK_META, Assets/Script/HapticManager.cs (log strings translated).
using UnityEngine;
using System.Runtime.InteropServices;

public class HapticManager : MonoBehaviour
{
    // Dll_DaVinci exports plain C functions, so the calling convention must be Cdecl.
    [DllImport("Dll_DaVinci", CallingConvention = CallingConvention.Cdecl)]
    public static extern int InitGeomagics([MarshalAs(UnmanagedType.LPStr)] string name);

    [DllImport("Dll_DaVinci", CallingConvention = CallingConvention.Cdecl)]
    public static extern void StartScheduler();

    [DllImport("Dll_DaVinci", CallingConvention = CallingConvention.Cdecl)]
    public static extern void GetPosition(int id, float[] pos);   // stylus tip, device frame [mm]

    [DllImport("Dll_DaVinci", CallingConvention = CallingConvention.Cdecl)]
    public static extern void GetRotation(int id, float[] rot);   // gimbal angles [rad]

    [DllImport("Dll_DaVinci", CallingConvention = CallingConvention.Cdecl)]
    public static extern int GetButtons(int id);                  // 1 = grey, 2 = white

    [DllImport("Dll_DaVinci", CallingConvention = CallingConvention.Cdecl)]
    public static extern void StopGeomagics();

    [DllImport("Dll_DaVinci", CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetForce(int id, float[] f);        // [N]

    public static HapticManager Instance { get; private set; }

    [Header("Device configuration (names from Touch Setup)")]
    public string deviceNameLeft = "Left Device";
    public string deviceNameRight = "Right Device";

    public int idLeft { get; private set; } = -1;
    public int idRight { get; private set; } = -1;

    private void Awake()
    {
        if (Instance == null) Instance = this;
        else Destroy(gameObject);
    }

    private void Start()
    {
        // OpenHaptics needs every device initialized before the shared scheduler starts.
        idLeft = InitGeomagics(deviceNameLeft);
        if (idLeft >= 0) Debug.Log($"<color=green>HD: {deviceNameLeft} initialized (ID:{idLeft}).</color>");
        else Debug.LogError($"<color=red>Error: {deviceNameLeft} not found.</color>");

        idRight = InitGeomagics(deviceNameRight);
        if (idRight >= 0) Debug.Log($"<color=green>HD: {deviceNameRight} initialized (ID:{idRight}).</color>");
        else Debug.LogError($"<color=red>Error: {deviceNameRight} not found.</color>");

        if (idLeft >= 0 || idRight >= 0)
        {
            StartScheduler();
            Debug.Log("<color=cyan>Haptic scheduler started.</color>");
        }
    }

    private void OnApplicationQuit()
    {
        StopGeomagics();
        Debug.Log("Geomagic devices disconnected.");
    }
}
