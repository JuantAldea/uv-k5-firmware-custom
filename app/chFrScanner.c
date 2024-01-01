
#include "app/app.h"
#include "app/chFrScanner.h"
#include "functions.h"
#include "misc.h"
#include "settings.h"

int8_t            gScanStateDir;
bool              gScanKeepResult;
bool              gScanPauseMode;

#ifdef ENABLE_SCAN_RANGES
uint32_t          gScanRangeStart;
uint32_t          gScanRangeStop;
#endif

typedef enum {
	SCAN_NEXT_CHAN_SCANLIST1 = 0,
	SCAN_NEXT_CHAN_SCANLIST2,
	SCAN_NEXT_CHAN_DUAL_WATCH,
	SCAN_NEXT_CHAN_MR,
	SCAN_NEXT_NUM
} scan_next_chan_t;

scan_next_chan_t	currentScanList;
uint32_t            initialFrqOrChan;
uint8_t           	initialCROSS_BAND_RX_TX;
uint32_t            lastFoundFrqOrChan;

static void NextFreqChannel(void);
static void NextMemChannel(void);
static void NextChannel(void);

void CHFRSCANNER_Start(const bool storeBackupSettings, const int8_t scan_direction)
{
	if (storeBackupSettings) {
		initialCROSS_BAND_RX_TX = gEeprom.CROSS_BAND_RX_TX;
		gEeprom.CROSS_BAND_RX_TX = CROSS_BAND_OFF;
		gScanKeepResult = false;
	}

	RADIO_SelectVfos();

	gNextMrChannel   = gRxVfo->CHANNEL_SAVE;
	currentScanList = SCAN_NEXT_CHAN_SCANLIST1;
	gScanStateDir    = scan_direction;

	gScheduleScanListen = false;
	gScanPauseMode = false;
	gRxReceptionMode = RX_MODE_NONE;

	if (storeBackupSettings) {
		lastFoundFrqOrChan = initialFrqOrChan;
		initialFrqOrChan = IS_MR_CHANNEL(gNextMrChannel) ? gRxVfo->CHANNEL_SAVE : gRxVfo->freq_config_RX.Frequency;
	}

	NextChannel();
}


void CHFRSCANNER_ContinueScanning(void)
{
	gScheduleScanListen = false;
	gScanPauseMode = false;
	gRxReceptionMode = RX_MODE_NONE;

	if (gCurrentFunction == FUNCTION_INCOMING && (IS_FREQ_CHANNEL(gNextMrChannel) || gCurrentCodeType == CODE_TYPE_OFF)) {
		APP_StartListening(gMonitor ? FUNCTION_MONITOR : FUNCTION_RECEIVE);
		return;
	}

	NextChannel();
}

void CHFRSCANNER_Found(void)
{
	gScanPauseDelayIn_10ms = 0;

	if (gEeprom.SCAN_RESUME_MODE == SCAN_RESUME_TO && !gScanPauseMode){
		gScanPauseDelayIn_10ms = scan_pause_delay_in_1_10ms;
		gScanPauseMode = true;
	}

	lastFoundFrqOrChan = IS_MR_CHANNEL(gRxVfo->CHANNEL_SAVE) ? gRxVfo->CHANNEL_SAVE : gRxVfo->freq_config_RX.Frequency;
	gScheduleScanListen = false;
	gScanKeepResult = true;
}

void CHFRSCANNER_Stop(void)
{
	if(initialCROSS_BAND_RX_TX != CROSS_BAND_OFF) {
		gEeprom.CROSS_BAND_RX_TX = initialCROSS_BAND_RX_TX;
		initialCROSS_BAND_RX_TX = CROSS_BAND_OFF;
	}

	gScanStateDir = SCAN_OFF;

	const uint32_t chFr = gScanKeepResult ? lastFoundFrqOrChan : initialFrqOrChan;
	const bool channelChanged = chFr != initialFrqOrChan;
	if (IS_MR_CHANNEL(gNextMrChannel)) {
		gEeprom.MrChannel[gEeprom.RX_VFO]     = chFr;
		gEeprom.ScreenChannel[gEeprom.RX_VFO] = chFr;
		RADIO_ConfigureChannel(gEeprom.RX_VFO, VFO_CONFIGURE_RELOAD);

		if(channelChanged) {
			SETTINGS_SaveVfoIndices();
			gUpdateStatus = true;
		}
	}
	else {
		gRxVfo->freq_config_RX.Frequency = chFr;
		RADIO_ApplyOffset(gRxVfo);
		RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
		if(channelChanged) {
			SETTINGS_SaveChannel(gRxVfo->CHANNEL_SAVE, gEeprom.RX_VFO, gRxVfo, 1);
		}
	}

	RADIO_SetupRegisters(true);
	gUpdateDisplay = true;
}


static void NextChannel()
{
#ifdef ENABLE_FASTER_CHANNEL_SCAN
	gScanPauseDelayIn_10ms = 9;  // 90ms .. <= ~60ms it misses signals (squelch response and/or PLL lock time) ?
#else
	gScanPauseDelayIn_10ms = scan_pause_delay_in_3_10ms;
#endif

	if (IS_FREQ_CHANNEL(gNextMrChannel)) {
		NextFreqChannel();  // switch to next frequency
	} else {
		NextMemChannel();    // switch to next channel
	}
}

static void NextFreqChannel(void)
{
#ifdef ENABLE_SCAN_RANGES
	if(gScanRangeStart) {
		gRxVfo->freq_config_RX.Frequency = APP_SetFreqByStepAndLimits(gRxVfo, gScanStateDir, gScanRangeStart, gScanRangeStop);
	} else
#endif
	{
		gRxVfo->freq_config_RX.Frequency = APP_SetFrequencyByStep(gRxVfo, gScanStateDir);
	}

	RADIO_ApplyOffset(gRxVfo);
	RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
	RADIO_SetupRegisters(true);

	gUpdateDisplay     = true;
}

static void NextMemChannel(void)
{
	static unsigned int prev_mr_chan = 0;
	const bool          enabled      = (gEeprom.SCAN_LIST_DEFAULT < 2) ? gEeprom.SCAN_LIST_ENABLED[gEeprom.SCAN_LIST_DEFAULT] : true;
	const int           chan1        = (gEeprom.SCAN_LIST_DEFAULT < 2) ? gEeprom.SCANLIST_PRIORITY_CH1[gEeprom.SCAN_LIST_DEFAULT] : -1;
	const int           chan2        = (gEeprom.SCAN_LIST_DEFAULT < 2) ? gEeprom.SCANLIST_PRIORITY_CH2[gEeprom.SCAN_LIST_DEFAULT] : -1;
	const unsigned int  prev_chan    = gNextMrChannel;
	unsigned int        chan         = 0;

	if (enabled)
	{
		switch (currentScanList)
		{
			case SCAN_NEXT_CHAN_SCANLIST1:
				prev_mr_chan = gNextMrChannel;

				if (chan1 >= 0)
				{
					if (RADIO_CheckValidChannel(chan1, false, 0))
					{
						currentScanList = SCAN_NEXT_CHAN_SCANLIST1;
						gNextMrChannel   = chan1;
						break;
					}
				}
				[[fallthrough]];
			case SCAN_NEXT_CHAN_SCANLIST2:
				if (chan2 >= 0)
				{
					if (RADIO_CheckValidChannel(chan2, false, 0))
					{
						currentScanList = SCAN_NEXT_CHAN_SCANLIST2;
						gNextMrChannel   = chan2;
						break;
					}
				}
				[[fallthrough]];

			// this bit doesn't yet work if the other VFO is a frequency
			case SCAN_NEXT_CHAN_DUAL_WATCH:
				// dual watch is enabled - include the other VFO in the scan
//				if (gEeprom.DUAL_WATCH != DUAL_WATCH_OFF)
//				{
//					chan = (gEeprom.RX_VFO + 1) & 1u;
//					chan = gEeprom.ScreenChannel[chan];
//					if (IS_MR_CHANNEL(chan))
//					{
//						currentScanList = SCAN_NEXT_CHAN_DUAL_WATCH;
//						gNextMrChannel   = chan;
//						break;
//					}
//				}

			default:
			case SCAN_NEXT_CHAN_MR:
				currentScanList = SCAN_NEXT_CHAN_MR;
				gNextMrChannel   = prev_mr_chan;
				chan             = 0xff;
				break;
		}
	}

	if (!enabled || chan == 0xff)
	{
		chan = RADIO_FindNextChannel(gNextMrChannel + gScanStateDir, gScanStateDir, (gEeprom.SCAN_LIST_DEFAULT < 2) ? true : false, gEeprom.SCAN_LIST_DEFAULT);
		if (chan == 0xFF)
		{	// no valid channel found
			chan = MR_CHANNEL_FIRST;
		}

		gNextMrChannel = chan;
	}

	if (gNextMrChannel != prev_chan)
	{
		gEeprom.MrChannel[    gEeprom.RX_VFO] = gNextMrChannel;
		gEeprom.ScreenChannel[gEeprom.RX_VFO] = gNextMrChannel;

		RADIO_ConfigureChannel(gEeprom.RX_VFO, VFO_CONFIGURE_RELOAD);
		RADIO_SetupRegisters(true);

		gUpdateDisplay = true;
	}

	if (enabled)
		if (++currentScanList >= SCAN_NEXT_NUM)
			currentScanList = SCAN_NEXT_CHAN_SCANLIST1;  // back round we go
}
