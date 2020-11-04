#include "SerialComm.h"

ushort SerialComm::CRC16(const byte* arr)
{
	int crc = 0xFFFF;
	for (int o = 0; o < BufferSize - 2; o++)
	{
		crc ^= (int)arr[o] << 8;
		for (int i = 0; i < 8; i++)
		{
			if ((crc & 0x8000) != 0)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
	}
	return (ushort)crc;
}
SerialComm::SerialComm(byte rx, byte tx, byte size) :
	RxAddress(rx), TxAddress(tx), BufferSize(size) {}

bool SerialComm::SetBuffer(const byte* input)
{
  UInt16 crc;
  crc.bytes[0] = input[BufferSize - 2];
  crc.bytes[1] = input[BufferSize - 1];
	if (SerialComm::CRC16(input) == crc.uint16)
	{
    memcpy(Buffer, input, BufferSize);
		return true;
	}
	return false;
}
void SerialComm::GetBuffer(byte* buff)
{
  UInt16 crc;
  crc.uint16 = SerialComm::CRC16(Buffer);
  Buffer[BufferSize - 2] = crc.bytes[0];
  Buffer[BufferSize - 1] = crc.bytes[1];
  memcpy(buff, Buffer, BufferSize);
}

ControllerConfig::ControllerConfig(byte rxAddr, byte txAddr) : SerialComm(rxAddr, txAddr, 26) {}

byte ControllerConfig::MinLvlGet(byte chnl)
{
	if (chnl > 7)
		return 0;
	return (byte)(Buffer[chnl] / 2.55f);
}

void ControllerConfig::MinLvlSet(byte chnl, byte value)
{
	if (chnl > 7)
		return;
	Buffer[chnl] = value;
}

void ControllerConfig::OverrideLvlSet(byte* chnls, byte val)
{
	Buffer[8] = val;
	Buffer[9] = 0; //Override mask
	Buffer[22] &= 0xef; //Clear 4th bit in config
	for (int x = 0; x < 3; x++)
	{
		if (chnls[x] > 8)
			continue;
		if (chnls[x] < 8)
			Buffer[9] |= (byte)(1 << chnls[x]);
		else if (chnls[x] == 8)
			Buffer[22] |= 1 << 4;
	}
}

byte ControllerConfig::FadeRateGet(byte link)
{
	if (link > 3)
		return 0;
	return Buffer[10 + link];
}
void ControllerConfig::FadeRateSet(byte link, byte value)
{
	if (link > 3)
		return;
	Buffer[10 + link] = value;
}
byte ControllerConfig::LinkDelayGet(byte link)
{
	if (link > 3)
		return 0;
	return Buffer[14 + link];
}
void ControllerConfig::LinkDelaySet(byte link, byte value)
{
	if (link > 3)
		return;
	Buffer[14 + link] = value;
}
void ControllerConfig::MSenLowTimeSet(byte value)
{
	if (value < 6)
		value = 0; //Zero means motion sensor isn't active
	Buffer[20] = value;
}
byte ControllerConfig::MSenLowTimeGet()
{
	return Buffer[20];
}

ControllerOTSet::ControllerOTSet(byte rxAddr, byte txAddr) : SerialComm(rxAddr, txAddr, 26) {}
void ControllerOTSet::ReplaceChOnTime(byte chnl, uint onTime, ushort swCnt)
{
	if (chnl > 8)
		return;
	UInt32 ot = { onTime };
	Buffer[0] = ot.bytes[0];
	Buffer[1] = ot.bytes[1];
	Buffer[2] = ot.bytes[2];
	Buffer[3] = ot.bytes[3];
	UInt16 sw = { swCnt };
	Buffer[10] = sw.bytes[0];
	Buffer[11] = sw.bytes[1];
	Buffer[9] = (byte)(chnl | 0x80);
	Buffer[22] = 0x80;
}

ControllerState::ControllerState(byte rxAddr, byte txAddr) : SerialComm(rxAddr, txAddr, 25) {}
uint ControllerState::GetTicks()
{
	UInt64 ticks;
	for (byte x = 0; x < 8; x++)
		ticks.bytes[x] = Buffer[x];
	return ticks.uint64 / 32;
}
byte ControllerState::GetChLevel(byte chnl)
{
	if (chnl > 8)
		return 0;
	return (byte)(Buffer[8 + chnl] / 2.55f);
}
byte ControllerState::GetLinkLevel(byte link)
{
	if (link > 3)
		return 0;
	return (byte)(Buffer[17 + link] / 2.55f);
}
bool ControllerState::IsLinkValid(byte link)
{
	if (link > 3)
		return false;
	return (Buffer[21] & (1 << link)) != 0;
}

byte ControllerState::MSenStateGet()
{
  return Buffer[22];
}

ControllerChOnTime::ControllerChOnTime(byte rxAddr, byte txAddr) : SerialComm(rxAddr, txAddr, 56) {}
uint ControllerChOnTime::GetOnTime(byte chnl)
{
	if (chnl > 8)
		return 0;
	UInt32 result;
	chnl *= 4;
	result.bytes[0] = Buffer[chnl++];
	result.bytes[1] = Buffer[chnl++];
	result.bytes[2] = Buffer[chnl++];
	result.bytes[3] = Buffer[chnl];
	return result.uint32;
}
ushort ControllerChOnTime::GetSwCount(byte chnl)
{
	if (chnl > 8)
		return 0;
	UInt16 result;
	chnl = 36 + chnl * 2;
	result.bytes[0] = Buffer[chnl++];
	result.bytes[1] = Buffer[chnl];
	return result.uint16;
}

VentilationConfig::VentilationConfig(byte rxAddr, byte txAddr) : SerialComm(rxAddr, txAddr, 11) {}
byte VentilationConfig::FanLevelGet()
{
	return (byte)(Buffer[0] / 1.47f);
}
void VentilationConfig::FanLevelSet(byte value)
{
	if (value > 99)
		Buffer[0] = 0xff;
	else
		Buffer[0] = (byte)(value * 1.47f);
}
byte VentilationConfig::MinRHGet()
{
	UInt16 rh;
	rh.bytes[0] = Buffer[1];
	rh.bytes[1] = Buffer[2];
	return rh.uint16 / 10;
}
void VentilationConfig::MinRHSet(byte value)
{
	UInt16 rh = { (ushort)(value * 10) };
	Buffer[1] = rh.bytes[0];
	Buffer[2] = rh.bytes[1];
}

VentilationState::VentilationState(byte rxAddr, byte txAddr) : SerialComm(rxAddr, txAddr, 17) {}
byte VentilationState::FanLevel()
{
	return (byte)(Buffer[0] / 1.47f);
}
ushort VentilationState::RPMFront()
{
	UInt16 rh;
	rh.bytes[0] = Buffer[1];
	rh.bytes[1] = Buffer[2];
	return rh.uint16;
}
ushort VentilationState::RPMRear()
{
	UInt16 rh;
	rh.bytes[0] = Buffer[3];
	rh.bytes[1] = Buffer[4];
	return rh.uint16;
}
ushort VentilationState::CurrentDraw()
{
	UInt16 rh;
	rh.bytes[0] = Buffer[5];
	rh.bytes[1] = Buffer[6];
	return rh.uint16;
}
ushort VentilationState::GetRH()
{
	UInt16 rh;
	rh.bytes[0] = Buffer[7];
	rh.bytes[1] = Buffer[8];
	return rh.uint16;
}
short VentilationState::GetTemperature()
{
	UInt16 rh;
	rh.bytes[0] = Buffer[11];
	rh.bytes[1] = Buffer[12];
	return (short)rh.uint16;
}
