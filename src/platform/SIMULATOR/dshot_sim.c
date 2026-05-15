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

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    (void)device;
    (void)motorConfig;
    return false;
}

#endif // USE_DSHOT
