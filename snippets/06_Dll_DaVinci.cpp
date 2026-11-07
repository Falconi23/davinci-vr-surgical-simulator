// Native OpenHaptics HD bridge for Unity, built as Dll_DaVinci.dll and P/Invoked by 04_HapticManager.cs.
// From the Visual Studio project Dll_DaVinci. Omitted: dllmain.cpp / pch.h / framework.h boilerplate.
#include "pch.h"
#include <HD/hd.h>
#include <map>
#include <vector>

// Keyed by small IDs so left and right arms share the same entry points.
static std::map<int, HHD> g_devices;
static int g_nextID = 0;
static bool g_schedulerStarted = false;

// Flat C ABI (no name mangling) for P/Invoke with CallingConvention.Cdecl.
extern "C" {

    // Name as configured in Touch Setup. Returns the ID, -1 for a null name, -2 if not found.
    __declspec(dllexport) int InitGeomagics(const char* deviceName) {
        if (deviceName == nullptr) return -1;

        HHD hHD = hdInitDevice(deviceName);
        if (hHD == HD_INVALID_HANDLE) {
            return -2;
        }

        hdMakeCurrentDevice(hHD);
        hdEnable(HD_FORCE_OUTPUT);

        int id = g_nextID++;
        g_devices[id] = hHD;
        return id;
    }

    // Call once, after all devices are initialized.
    __declspec(dllexport) void StartScheduler() {
        if (!g_schedulerStarted && g_devices.size() > 0) {
            hdStartScheduler();
            g_schedulerStarted = true;
        }
    }

    // Device frame [mm]. Every accessor brackets the read in a frame on its own device.
    __declspec(dllexport) void GetPosition(int id, float* p) {
        if (!g_schedulerStarted || g_devices.count(id) == 0 || p == nullptr) return;

        hdMakeCurrentDevice(g_devices[id]);
        hdBeginFrame(g_devices[id]);

        double pos[3];
        hdGetDoublev(HD_CURRENT_POSITION, pos);

        hdEndFrame(g_devices[id]);

        p[0] = (float)pos[0];
        p[1] = (float)pos[1];
        p[2] = (float)pos[2];
    }

    // Gimbal angles [rad].
    __declspec(dllexport) void GetRotation(int id, float* r) {
        if (!g_schedulerStarted || g_devices.count(id) == 0 || r == nullptr) return;

        hdMakeCurrentDevice(g_devices[id]);
        hdBeginFrame(g_devices[id]);

        double rot[3];
        hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, rot);

        hdEndFrame(g_devices[id]);

        r[0] = (float)rot[0];
        r[1] = (float)rot[1];
        r[2] = (float)rot[2];
    }

    // Bit 0: grey (front) button, bit 1: white (rear).
    __declspec(dllexport) int GetButtons(int id) {
        if (!g_schedulerStarted || g_devices.count(id) == 0) return 0;

        hdMakeCurrentDevice(g_devices[id]);
        hdBeginFrame(g_devices[id]);

        int buttons = 0;
        hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);

        hdEndFrame(g_devices[id]);
        return buttons;
    }

    __declspec(dllexport) void StopGeomagics() {
        if (g_schedulerStarted) {
            hdStopScheduler();
            g_schedulerStarted = false;
        }
        for (auto const& [id, hHD] : g_devices) {
            hdDisableDevice(hHD);
        }
        g_devices.clear();
        g_nextID = 0;
    }

    // Cartesian force [N], applied at hdEndFrame.
    __declspec(dllexport) void SetForce(int id, float* f) {
        if (!g_schedulerStarted || g_devices.count(id) == 0 || f == nullptr) return;

        hdMakeCurrentDevice(g_devices[id]);
        hdBeginFrame(g_devices[id]);

        double force[3];
        force[0] = (double)f[0];
        force[1] = (double)f[1];
        force[2] = (double)f[2];

        hdSetDoublev(HD_CURRENT_FORCE, force);

        hdEndFrame(g_devices[id]);
    }

}
