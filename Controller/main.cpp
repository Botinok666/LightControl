/* Controller.cpp
 * Created: 12.01.2018 11:30:55
 * Version: 1.4 */

#include "LC.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>

volatile int16_t gLevels[9] = {0};
volatile uint8_t gLevelChg = 0, rxMode = 0, rxMark;
volatile uint8_t DSI8xFrames[19];
volatile uint8_t *framePtr;

union i16i8
{
	uint16_t ui16;
	uint8_t ui8[2];
	int16_t i16;
	int8_t i8[2];
};

struct systemConfig
{
	uint8_t minLvl[8];
	uint8_t overrideLvl, overrideCfg; //Config: bits 0-7: mask for channels 0-7
	uint8_t fadeRate[4], linkDelay[4];
	uint8_t msenOnTime, msenOnLvl, msenLowTime, msenLowLvl; //On Level not used since v1.4
	uint8_t groupConf; //bit 4: override channel 8, bit 3: save to EEPROM
	uint8_t rtcCorrect; //Value in ppm, sign-and-magnitude representation

	uint16_t CRC16;
} validConf;

struct systemState
{
	uint64_t sysTick;
	uint8_t setLevels[9];
	uint8_t linkLevels[4];
	uint8_t linksMask; //Bits 0-3: valid analog signal from link 1-4; bits 4-7: non-zero level at link 1-4
	uint8_t msenLevel;
	
	uint16_t CRC16;
} sysState;

struct channelsOnTime
{
	uint32_t linkOnTime[9];
	uint16_t linkSwCnt[9];

	uint16_t CRC16;
} channelOT;

uint8_t iobuf[MAX(sizeof(systemConfig), MAX(sizeof(systemState), sizeof(channelsOnTime)))];

class LCport
{
private:
	uint8_t _linkCnt, _linkNum, _link[3]; //Positions in common array for light levels
	uint8_t _lvl[3]; //Actual light levels, range: 0…255
	uint8_t _fadeRate; //Steps per second, range: 32…160 (settling time 0…100%: 8…1.6s)
	int16_t _linkDelay; //Delay between links, range: 1…64 (time: 0.031…2s)
	uint8_t _chActMask; //Mask for driving channel activity LED
	uint64_t _tickLastChg, _onTimeStamp;

public:
	bool direction, msenCtrl; //Direction of group delay, is it controlled by motion sense
	LCport(uint8_t num, uint8_t count, uint8_t posA, uint8_t posB, uint8_t posC)
	{
		_linkCnt = count;
		_link[0] = posA;
		_link[1] = posB;
		_link[2] = posC;
		_linkNum = num;
		_chActMask = 0x10 << num;
	};

	void setParams()
	{
		uint8_t pos, min = validConf.fadeRate[_linkNum], max = validConf.linkDelay[_linkNum];
		if (min < 32)
			min = 32;
		_fadeRate = (min > 160) ? 160 : min;
		_linkDelay = (max > 63) ? 64 : max + 1;
		for (uint8_t i = 0; i < _linkCnt; i++)
		{
			pos = _link[i];
			if ((pos < 8 && (validConf.overrideCfg & (1 << pos))) || (pos >= 8 && (validConf.groupConf & (1 << 4))))
			{
				if (_lvl[i] == gLevels[pos]) //Refresh saved ticks value only if no dimming in the process right now
					_tickLastChg = sysState.sysTick;
				_lvl[i] = validConf.overrideLvl;
			}
		}
	}

	void setLevel(uint8_t level) //Accepts level in the range 0…255
	{
		bool dimInProcess = false;
		uint8_t oLvl = sysState.linkLevels[_linkNum];
		int16_t tempLvl = (int16_t)oLvl - level;
		if (-3 < tempLvl && tempLvl < 3 && ((level == 0) == (0 == oLvl)))
			return;
		if (!msenCtrl)
			direction = oLvl > level;
		sysState.linkLevels[_linkNum] = level;

		for (uint8_t i = 0; i < _linkCnt; i++)
		{
			uint8_t j = _link[i];
			uint8_t min = j < 8 ? validConf.minLvl[j] : 0x05;
			dimInProcess |= (_lvl[i] != gLevels[j]);
			if (!level) //Set zero level directly
				_lvl[i] = 0;
			else //Convert non-zero value to the actual range
			{
				uint16_t temp = level;
				temp *= 0xFF - min;
				_lvl[i] = (uint8_t)(temp >> 8) + min;
			}
		}
		if (!dimInProcess) //Refresh saved ticks value only if no dimming in the process right now
			_tickLastChg = sysState.sysTick;
	}

	void updateLevel()
	{
		int16_t ticksEl = (int16_t)(sysState.sysTick - _tickLastChg); //Elapsed ticks since beginning of dim
		int16_t delta = (((ticksEl + 1) * _fadeRate) >> 5) - ((ticksEl * _fadeRate) >> 5); //Number of steps
		PORTC.OUTCLR = _chActMask;
		for (int8_t i = 0; i < _linkCnt; i++)
		{
			uint8_t s = direction ? i : _linkCnt - i - 1; //Direction '1' means forward
			uint8_t j = _link[s];
			uint8_t min = j < 8 ? validConf.minLvl[j] : 0x05;
			if (_lvl[s] < min)
				_lvl[s] = 0;
			int16_t tempLvl = gLevels[j] - _lvl[s]; //Difference between actual and set levels
			bool changed = false;
			if (tempLvl && ticksEl >= i * _linkDelay)
			{
				if (tempLvl > 0) //Level needs to be lowered
				{
					tempLvl -= ((tempLvl > delta) ? delta : tempLvl) - _lvl[s];
					if (tempLvl < min) //Actual level became zero
					{
						changed = true;
						msenCtrl = false;
						tempLvl = -((int16_t)_fadeRate << 2); //Subtract 4x fade steps, so off/on delay will be 4s
						channelOT.linkOnTime[j] += (uint32_t)(sysState.sysTick - _onTimeStamp) >> 5; //Add seconds
						channelOT.linkSwCnt[j]++; //Increment switch counter
					}
				}
				else //Level needs to be raised
				{
					tempLvl += ((-tempLvl > delta) ? delta : -tempLvl) + _lvl[s];
					if (!gLevels[j] && tempLvl > 0) //Lamp has been switched on - remember ticks
					{
						if (tempLvl < min)
							tempLvl = min;
						_onTimeStamp = sysState.sysTick;
					}
				}
				gLevels[j] = tempLvl;
				if (tempLvl >= 0 || changed)
				{
					gLevelChg |= 1 << j;
					PORTC.OUTSET = _chActMask; //Switch on activity LED
				}
			}
		}
	}
} links[4] = { LCport(0, 3, 7, 6, 5), LCport(1, 3, 4, 3, 2),
#ifdef BOARD_A
LCport(2, 1, 1, 0, 0),
#else
LCport(2, 2, 1, 0, 0),
#endif
LCport(3, 1, 8, 0, 0) };

class Msen
{
private:
	bool cntDown, ltEnt;
	uint8_t _linkMask;
	LCport *_linkAddr;
	uint8_t _lvl;
	uint16_t _onTime, _fTime;
	uint64_t _onTimeStamp;
public:
	Msen(uint8_t actLink)
	{
		_linkMask = 0x10 << actLink;
		_linkAddr = &links[actLink];
	};
	void setParams()
	{
		_onTime = validConf.msenOnTime > 5 ? validConf.msenOnTime : 6;
		_fTime = _onTime + validConf.msenLowTime;
	}
	void setLevel(uint8_t level)
	{
		sysState.msenLevel = level;
		if (level < MSEN_VALID_MIN || !validConf.msenOnTime || (sysState.linksMask & _linkMask))
		{
			_lvl = 0; //MSEN disconnected, disabled (by zero on time) or there is non-zero level from link
			_linkAddr->msenCtrl = cntDown = false;
			return;
		}
		if (level > MSEN_SEN1_TRIG)
		{
			if (_lvl < MSEN_SEN1_TRIG) //Off to on transition
			{
				_linkAddr->msenCtrl = true;
				_linkAddr->direction = level > MSEN_SEN2_TRIG;
				uint8_t onLvl = MAX(sysState.linkLevels[0], sysState.linkLevels[1]); 
				//Start from max light level from any of 2 rooms
				_linkAddr->setLevel(onLvl < validConf.msenLowLvl ? validConf.msenLowLvl : onLvl);
				ltEnt = cntDown = false;				
			}
			_lvl = level;
		}
		else
		{
			if (_lvl > level) //On to off transition
			{
				_onTimeStamp = sysState.sysTick;
				cntDown = true;
			}
			_lvl = MSEN_VALID_MIN;
		}
	}
	void updateLevel()
	{
		if (cntDown)
		{
			uint16_t ton = (uint16_t)(sysState.sysTick - _onTimeStamp) >> 5;
			if (ton > _fTime)
			{
				ltEnt = cntDown = false;
				_linkAddr->setLevel(0); //Both on and low time are expired, turn off the lamp
			}
			else if (!ltEnt && ton > _onTime)
			{
				ltEnt = true;
				_linkAddr->direction ^= true;
				_linkAddr->setLevel(validConf.msenLowLvl);
			}
		}
	}
} msenCh = Msen(2);

channelsOnTime EEMEM savedCOT;
systemConfig EEMEM savedConfig = {
{5,5,5,5,5,5,5,5},	//Min levels
0, 0,	//Override level and config
{96,96,96,160}, {28,28,28,1},	//Fade rate 2.6s, link delay 0.87s
0, 255, 30, 96,	//Motion sensor on time, on level, low time, low level
0, 0, 0};	//groupConfig, rtcCorrect, CRC16

void inline UCTXen()
{
	PORTC.OUTSET = PIN1_bm;
	_delay_us(78);
}

uint16_t CalculateCRC16(void *arr, int8_t count)
{
	uint8_t *ptr = (uint8_t*)arr;
	CRC.CTRL = CRC_RESET_RESET1_gc;
	CRC.CTRL = CRC_SOURCE_IO_gc;
	while (--count >= 0)
		CRC.DATAIN = *ptr++;
	i16i8 result;
	CRC.STATUS = CRC_BUSY_bm;
	result.ui8[0] = CRC.CHECKSUM0;
	result.ui8[1] = CRC.CHECKSUM1;
	CRC.CTRL = CRC_SOURCE_DISABLE_gc;
	return result.ui16;
}

void ApplyConfig()
{
	systemConfig *temp = (systemConfig*)iobuf;
	//Both bits 7 are set: one of the on-time counters must be rewritten
	if ((temp->groupConf & 0x80) && (temp->overrideCfg & 0x80))
	{
		uint8_t j = temp->overrideCfg & 0xF; //Update on-time of link
		channelOT.linkOnTime[j] = *((uint32_t*)temp->minLvl);
		channelOT.linkSwCnt[j] = *((uint16_t*)temp->fadeRate);
		return;
	}
	memcpy(&validConf, iobuf, sizeof(systemConfig));
	for (uint8_t i = 0; i < 4; i++)
		links[i].setParams();
	msenCh.setParams();
	if (validConf.groupConf & (1 << 3)) //Bit 3 is set: save config to EEPROM
		eeprom_update_block(&validConf, &savedConfig, sizeof(systemConfig));
	RTC.CALIB = validConf.rtcCorrect;
}

ISR(RTC_OVF_vect)
{
	sysState.sysTick++;
	uint8_t i;
	for (i = 0; i < 4; i++)
		links[i].updateLevel();
	msenCh.updateLevel();
	for (i = 0; i < 9; i++)
		sysState.setLevels[i] = (gLevels[i] < 1) ? 0 : (uint8_t)gLevels[i];
	DSI8xFrames[0] = gLevelChg; //DSI start bit
	for (i = 0; i < 8; i++) //DSI frame bits 0-7
	{
		uint8_t tmp0 = 0, tmp1 = 0, j;
		for (j = 0; j < 8; j++) //Levels
		{
			tmp1 >>= 1;
			tmp0 >>= 1;
			if (gLevelChg & (1 << j)) //Particular level has been changed
			{
				if (sysState.setLevels[j] & (1 << i)) //Set upper half-bit
					tmp1 |= 0x80;
				else
					tmp0 |= 0x80; //Set lower half-bit (one-zero transition)
			}
		}
		j = (8 - i) << 1;
		DSI8xFrames[j - 1] = tmp1;
		DSI8xFrames[j] = tmp0; //Manchester coded
	}
	DSI8xFrames[17] = 0; //DSI stop bit
	gLevelChg = 0; //Clear flags for changed levels
	if (sysState.setLevels[8] > 0) //On/off channel processing
		PORTA.OUTSET = PIN7_bm;
	else
		PORTA.OUTCLR = PIN7_bm;

	if ((char)sysState.sysTick % 4 == 0) //Initialize conversion sequence 8 times per second
	{
		ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
		ADCA.CTRLA |= ADC_START_bm; //Start conversion from pin 1 (channel 1)
	}

	if (!((uint8_t)sysState.sysTick & 0x1F))
		PORTC.OUTTGL = PIN0_bm; //Heartbeat LED

	if (((uint32_t)sysState.sysTick & 0x7FFFF) == 0) //Save state to EEPROM every 4.5 hrs
		eeprom_update_block(&channelOT, &savedCOT, sizeof(channelsOnTime));

	if (rxMode == SetConfig && 2 < ++rxMark) //We are currently receiving data packet
	{
		EDMA.CH0.CTRLA = 0;
		rxMode = 0; //Packet considered lost
		USARTC0.CTRLB |= USART_MPCM_bm; //Set MPCM bit
		USARTC0.CTRLA = USART_DRIE_bm | USART_RXCINTLVL_HI_gc;
		while (EDMA.CH0.CTRLB & EDMA_CH_CHBUSY_bm);
	}
	framePtr = DSI8xFrames;
	TCD5.INTFLAGS |= TC5_OVFIF_bm;
	TCD5.INTCTRLA = TC_OVFINTLVL_HI_gc;
}

ISR(ADCA_CH0_vect)
{
	static uint8_t sAdcCnt = 0;
	int16_t result = ADCA.CH0RES;
	if (sAdcCnt < 4 && LINK_VALID_MIN < result && result < LINK_VALID_MAX)
	{
		sysState.linksMask |= (1 << sAdcCnt); //Set 'valid' bit
		if (result < LINK_ON_BARRIER)
		{
			result = 0;
			if (sysState.linksMask & (0x10 << sAdcCnt)) //Previous level was non-zero
				links[sAdcCnt].setLevel(0);
			sysState.linksMask &= ~(0x10 << sAdcCnt); //Clear 'non-zero' bit
		}
		else
		{
			result = (result - LINK_1PERC_CODE) / LINK_SCALE;
			if (result < 5)
				result = 5;
			if (result > 255)
				result = 255;
			sysState.linksMask |= (0x10 << sAdcCnt); //Set 'non-zero' bit
			links[sAdcCnt].setLevel((uint8_t)result);
		}
	}
	else if (sAdcCnt < 4)
		sysState.linksMask &= ~(0x11 << sAdcCnt); //Clear 'valid' and 'non-zero' bits
	else //sAdcCnt == 4: MSEN channel
		msenCh.setLevel(result < 0 ? 0 : result >> 6);

	sAdcCnt++;
	ADCA.CH0.MUXCTRL = (sAdcCnt + 1) << ADC_CH_MUXPOS_gp;
	if (sAdcCnt < 5)
		ADCA.CTRLA |= ADC_START_bm;
	else
		sAdcCnt = 0;
}

ISR(USARTC0_RXC_vect) //Data received from RS485
{
	uint8_t data = USARTC0.DATA;
	if (USARTC0.CTRLB & USART_MPCM_bm) //Address listening mode
	{
		if (CmdLC <= data && data <= CmdUC)
		{
			USARTC0.CTRLB &= ~USART_MPCM_bm; //Clear MPCM bit
			rxMode = data;
			if (data == SetConfig)
			{
				rxMark = 0;
				EDMA.CH0.ADDR = (register16_t)iobuf;
				EDMA.CH0.TRFCNT = sizeof(systemConfig); //Bytes to receive into iobuf
				EDMA.CH0.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_SINGLE_bm;
				USARTC0.CTRLA = USART_RXCINTLVL_OFF_gc; //Disable interrupt
			}
			else //Data transmit request
			{
				UCTXen();
				if (rxMode == GetConfig)
				{
					EDMA.CH1.TRFCNT = sizeof(systemConfig) + 3;
					EDMA.CH1.ADDR = (register16_t)&validConf;
				}
				else if (rxMode == GetStatus) //Get state
				{
					memcpy(iobuf, &sysState, sizeof(systemState));
					((systemState*)iobuf)->CRC16 = CalculateCRC16(iobuf, sizeof(systemState) - 2);
					EDMA.CH1.TRFCNT = sizeof(systemState) + 3;
					EDMA.CH1.ADDR = (register16_t)iobuf;
				}
				else //Get on time
				{
					memcpy(iobuf, &channelOT, sizeof(channelsOnTime));
					((channelsOnTime*)iobuf)->CRC16 = CalculateCRC16(iobuf, sizeof(channelsOnTime) - 2);
					EDMA.CH1.TRFCNT = sizeof(channelsOnTime) + 3;
					EDMA.CH1.ADDR = (register16_t)iobuf;
				}
				EDMA.CH1.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_SINGLE_bm;
			}
		}
		else
			rxMode = 0;
	}
}

ISR(EDMA_CH0_vect)
{
	rxMode = 0;
	USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_MPCM_bm; //Set MPCM bit
	USARTC0.CTRLA = USART_DRIE_bm | USART_RXCINTLVL_MED_gc;
	uint16_t crc = CalculateCRC16(iobuf, sizeof(systemConfig) - 2);
	if (crc == ((systemConfig*)iobuf)->CRC16)
		ApplyConfig();
	EDMA.CH0.CTRLA = 0;
	EDMA.CH0.CTRLB = EDMA_CH_TRNIF_bm | EDMA_CH_ERRIF_bm | EDMA_CH_TRNINTLVL_LO_gc;
}

ISR(EDMA_CH1_vect) //Packet has been sent completely over RS485
{
	UCRXen(); //Set bus in the idle state
	rxMode = 0;
	USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_MPCM_bm; //Set MPCM bit
	EDMA.CH1.CTRLA = 0;
	EDMA.CH1.CTRLB = EDMA_CH_TRNIF_bm | EDMA_CH_ERRIF_bm | EDMA_CH_TRNINTLVL_LO_gc;
}

ISR(TCD5_OVF_vect)
{
	PORTD.OUT = *framePtr++;
	if (framePtr == DSI8xFrames + sizeof(DSI8xFrames) - 1)
		TCD5.INTCTRLA = TC_OVFINTLVL_OFF_gc;
	TCD5.INTFLAGS = TC5_OVFIF_bm;
}

inline void mcuInit()
{
	//Port A configuration: 0-5 analog inputs, 7 inverted output
	PORTA.DIRSET = PIN7_bm;
	PORTCFG.MPCMASK = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm;
	PORTA.PIN0CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN7CTRL = PORT_OPC_TOTEM_gc | PORT_INVEN_bm;
	//Port C configuration: 0-1, 3-7 outputs
	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
	//Port D configuration: 0-7 inverted outputs
	PORTD.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
	PORTCFG.MPCMASK = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
	PORTD.PIN0CTRL = PORT_OPC_TOTEM_gc | PORT_INVEN_bm | PORT_ISC_INPUT_DISABLE_gc;
	//Clock configuration: 32MHz RC with DFLL from XOSC 32768Hz
	OSC.XOSCCTRL = OSC_XOSCSEL_32KHz_gc;
	OSC.CTRL |= OSC_RC32MEN_bm | OSC_XOSCEN_bm;
	while (!(OSC.STATUS & OSC_XOSCRDY_bm));
	while (!(OSC.STATUS & OSC_RC32MRDY_bm));
	OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc;
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
	OSC.CTRL = OSC_RC32MEN_bm | OSC_XOSCEN_bm; //Disable unused clock sources
	CLK.RTCCTRL = CLK_RTCSRC_TOSC_gc | CLK_RTCEN_bm; //1024Hz RTC clock source
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	CCP = CCP_IOREG_gc;
	CLK.LOCK = CLK_LOCK_bm; //Disable further changes in clock system
	//PMIC: enable interrupts
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	//RTC configuration: 1024 / 2 = 512Hz clock, medium level interrupt
	RTC.PER = 15; //32 overflows per second
	RTC.INTCTRL = RTC_OVFINTLVL_MED_gc;
	RTC.CTRL = RTC_PRESCALER_DIV2_gc;
	RTC.CNT = 0;
	//USARTC configuration: start / 9 data / 2 stop; 76.8kbps baud, multi-processor communication
	USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_MPCM_bm;
	USARTC0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_SBMODE_bm | USART_CHSIZE_9BIT_gc;
	USARTC0.BAUDCTRLA = 12 << USART_BSEL_gp;
	USARTC0.BAUDCTRLB = 1 << USART_BSCALE_gp;
	USARTC0.CTRLA = USART_DRIE_bm | USART_RXCINTLVL_MED_gc;
	//ADC configuration: 1MHz, 15bit oversampling
	ADCA.CTRLA = ADC_ENABLE_bm;
	ADCA.CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_MT12BIT_gc; //Signed mode
	ADCA.REFCTRL = ADC_REFSEL_AREFA_gc;
	ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc;
	ADCA.SAMPCTRL = 7 << ADC_SAMPVAL_gp;
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc;
	ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_LO_gc;
	ADCA.CH0.AVGCTRL = ADC_CH_SAMPNUM_16X_gc; //This will produce 15 bit result
	//TCD5 configuration: 500kHz, 1199Hz overflow rate
	TCD5.CTRLA = TC_CLKSEL_DIV64_gc;
	TCD5.CTRLB = TC_WGMODE_NORMAL_gc;
	TCD5.PERBUF = 417;
	TCD5.CTRLGCLR = TC5_STOP_bm;
	//EDMA peripheral channel 0: USARTC read transfer
	EDMA.CH0.CTRLB = EDMA_CH_TRNINTLVL_LO_gc;
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_USARTC0_RXC_gc;
	//EDMA peripheral channel 1: USARTC write transfer
	EDMA.CH1.CTRLB = EDMA_CH_TRNINTLVL_LO_gc; //Low-level interrupt
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH1.TRIGSRC = EDMA_CH_TRIGSRC_USARTC0_DRE_gc;
	//EDMA: 1 standard and 2 peripheral channels
	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_CHMODE_STD2_gc | EDMA_DBUFMODE_DISABLE_gc | EDMA_PRIMODE_RR0123_gc;
	//Power reduction
	PR.PRGEN = PR_XCL_bm | PR_EVSYS_bm;
	PR.PRPA = PR_DAC_bm | PR_AC_bm;
	PR.PRPC = PR_TWI_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC5_bm | PR_TC4_bm;
	PR.PRPD = PR_USART0_bm;
	sei();
}

int main(void)
{
	mcuInit();
	eeprom_read_block(&channelOT, &savedCOT, sizeof(channelsOnTime));
	eeprom_read_block(iobuf, &savedConfig, sizeof(systemConfig) - 2);
	((systemConfig*)iobuf)->overrideCfg = ((systemConfig*)iobuf)->groupConf = 0; //Ignore saved overrides
	((systemConfig*)iobuf)->CRC16 = CalculateCRC16(iobuf, sizeof(systemConfig) - 2);
	ApplyConfig();
    /* Replace with your application code */
    while (1) {}
}
