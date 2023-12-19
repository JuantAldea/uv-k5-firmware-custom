/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include "audio.h"
#include "functions.h"
#include "misc.h"
#include "settings.h"

#include "app/chFrScanner.h"
#include "app/scanner.h"
#ifdef ENABLE_FMRADIO
	#include "app/fm.h"
#endif
#include "bsp/dp32g030/gpio.h"
#include "driver/backlight.h"
#include "driver/gpio.h"
#include "helper/battery.h"

#define DECREMENT(cnt) \
	do {               \
		if (cnt > 0)   \
			cnt--;     \
	} while (0)

#define DECREMENT_AND_TRIGGER(cnt, flag) \
	do {                                 \
		if (cnt > 0)                     \
			if (--cnt == 0)              \
				flag = true;             \
	} while (0)

volatile uint32_t gGlobalSysTickCounter;


// we come here every 10ms
void SystickHandler(void)
{
	gGlobalSysTickCounter++;

	gNextTimeslice = true;

	DECREMENT(gFoundCDCSSCountdown_10ms);

	DECREMENT(gFoundCTCSSCountdown_10ms);

	DECREMENT(boot_counter_10ms);

	DECREMENT_AND_TRIGGER(gTailNoteEliminationCountdown_10ms, gFlagTailNoteEliminationComplete);

	if (gCurrentFunction == FUNCTION_FOREGROUND) {
		DECREMENT_AND_TRIGGER(gBatterySaveCountdown_10ms, gSchedulePowerSave);
	}

	if (gCurrentFunction == FUNCTION_POWER_SAVE) {
		DECREMENT_AND_TRIGGER(gPowerSave_10ms, gPowerSaveCountdownExpired);
	}

	if (gScanStateDir != SCAN_OFF && gCurrentFunction != FUNCTION_MONITOR && gCurrentFunction != FUNCTION_TRANSMIT) {
		DECREMENT_AND_TRIGGER(gScanPauseDelayIn_10ms, gScheduleScanListen);

		if(!gCssBackgroundScan && gEeprom.DUAL_WATCH != DUAL_WATCH_OFF && gCurrentFunction != FUNCTION_RECEIVE) {
			DECREMENT_AND_TRIGGER(gDualWatchCountdown_10ms, gScheduleDualWatch);
		}
	}

#ifdef ENABLE_VOX
	DECREMENT(gVoxStopCountdown_10ms);
#endif

#ifdef ENABLE_VOICE
	DECREMENT_AND_TRIGGER(gCountdownToPlayNextVoice_10ms, gFlagPlayQueuedVoice);
#endif

#ifdef ENABLE_FMRADIO
	if (gFM_ScanState != FM_SCAN_OFF && gCurrentFunction != FUNCTION_MONITOR) {
		if (gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_RECEIVE) {
			DECREMENT_AND_TRIGGER(gFmPlayCountdown_10ms, gScheduleFM);
		}
	}
#endif

#ifdef ENABLE_NOAA
	DECREMENT(gNOAACountdown_10ms);

	if (gScanStateDir == SCAN_OFF && !gCssBackgroundScan && gEeprom.DUAL_WATCH == DUAL_WATCH_OFF) {
		if (gIsNoaaMode && gCurrentFunction != FUNCTION_MONITOR && gCurrentFunction != FUNCTION_TRANSMIT) {
			if (gCurrentFunction != FUNCTION_RECEIVE) {
				DECREMENT_AND_TRIGGER(gNOAA_Countdown_10ms, gScheduleNOAA);
			}
		}
	}
#endif

	if ((gGlobalSysTickCounter & 3) == 0) {
		gNextTimeslice40ms = true;
	}

	if ((gGlobalSysTickCounter % 50) == 0) {
		gNextTimeslice_500ms = true;

		DECREMENT_AND_TRIGGER(gTxTimerCountdown_500ms, gTxTimeoutReached);

		DECREMENT(gSerialConfigCountDown_500ms);
	}

}
