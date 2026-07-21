/*
 * DSHOT telemetry stubs for SIMULATOR platform.
 *
 * Most DSHOT functions are now compiled from drivers/dshot.c (via USE_DSHOT +
 * USE_DSHOT_TELEMETRY defines).  This file only provides the platform-specific
 * globals and functions that dshot.c references but no SIMULATOR driver supplies.
 *
 * Motor eRPM values are injected by the simulator orchestrator via
 * dshotTelemetryState.motorState[].telemetryData[].
 */

#include "platform.h"

#ifdef USE_DSHOT

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/motor_impl.h"

/* Global referenced by dshot.c, rpm_filter.c, dyn_notch.c, etc. */
bool useDshotTelemetry = false;

/* Globals referenced by CLI dshot telemetry info. */
dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;
uint32_t readDoneCount = 0;
uint32_t inputStampUs = 0;

/* Platform stubs — not applicable to SIMULATOR. */
bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
{
    (void)motorDevConfig;
    return false;
}

/* Minimal SIMULATOR DShot motor DEVICE (2026-07-11).
 *
 * This used to be a stub returning false, which left motorDevice.initialized
 * = false for the production DShot protocol -> motorIsEnabled() false forever
 * -> BF's msp.c served MSP_MOTOR as ZEROS to the modem CA (while lockstep
 * harnesses read the mixer's motor[] directly and never noticed). Harmless
 * while the CA speed model was pitch-dominant; after the motor-dominant
 * airspeed refit it collapsed the HITL force-balance speed ~50% -> position
 * DR under-propagation + heading-regression starvation (run-proven HITL
 * 2026-07-11 145811: NAV spd 20-28 vs truth 44-47).
 *
 * The device layer here only exists to make the MSP surface faithful to the
 * real aircraft (where MSP_MOTOR serves real values): enable/isMotorEnabled
 * report true and the convert functions are the real DShot ones, so
 * msp.c returns motorConvertToExternal(motor[i]) exactly like flight
 * hardware. write/updateComplete are no-ops — the sim physics reads the
 * mixer output directly (bf_wrapper), not the device. */

static uint8_t dshotSimMotorCount = 0;

static bool dshotSimEnable(void) { return true; }
static void dshotSimDisable(void) { }
static bool dshotSimIsMotorEnabled(unsigned index)
{
    return index < dshotSimMotorCount;
}
static void dshotSimWrite(uint8_t index, float value)
{
    (void)index; (void)value;
}
static void dshotSimWriteInt(uint8_t index, uint16_t value)
{
    (void)index; (void)value;
}
static void dshotSimUpdateComplete(void) { }
static void dshotSimShutdown(void) { }

static const motorVTable_t dshotSimVTable = {
    .postInit = motorPostInitNull,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .enable = dshotSimEnable,
    .disable = dshotSimDisable,
    .isMotorEnabled = dshotSimIsMotorEnabled,
    .decodeTelemetry = motorDecodeTelemetryNull,
    .write = dshotSimWrite,
    .writeInt = dshotSimWriteInt,
    .updateComplete = dshotSimUpdateComplete,
    .shutdown = dshotSimShutdown,
    .requestTelemetry = NULL,
    .isMotorIdle = NULL,
    .getMotorIO = NULL,
};

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    (void)motorConfig;
    if (!device) {
        return false;
    }
    dshotSimMotorCount = device->count;
    device->vTable = &dshotSimVTable;
    return true;
}

#endif // USE_DSHOT
