#pragma once
#include <Arduino.h>
typedef unsigned char byte;
typedef unsigned short ushort;
typedef unsigned int uint;

//! Union for converting between UInt64 and a byte array
union UInt64
{
	uint64_t uint64;
	byte bytes[8];
};
//! Union for converting between UInt32 and a byte array
union UInt32
{
	uint uint32;
	byte bytes[4];
};
//! Union for converting between UInt16 and a byte array
union UInt16
{
	ushort uint16;
	byte bytes[2];
};
//! Structure with parameters for lighting control in a room
struct RoomParams
{
	byte roomID, //!< Room ID in array of rooms
		controllerID, //!< Controller ID in array of floors
		linkID; //!< Link ID for room in the controller
	byte channels[3]; //!< Channel for each lighting fixture in the controller
};
/*! Class for serial communication */
class SerialComm
{
private:
	//! CRC16-CCITT calculation
	/*! This is the CRC16-CCITT. HEX: 0x1021; initial value: 0xFFFF
		\param arr Byte array to calculate CRC from
		\returns CRC16 of given array (except two last bytes) */
	ushort CRC16(const byte* arr);
protected:
	//! Class constructor
	/*! \param rx Set endpoint address for reading data
		\param tx Set endpoint address for writing data
		\param size Set endpoint buffer size */
	SerialComm(byte rx, byte tx, byte size);
	byte Buffer[64] = { 0 }; //!< Endpoint buffer (all internal fields combined in one array)
public:
	const byte RxAddress; //!< Endpoint address for reading data (read-only)
	const byte TxAddress; //!< Endpoint address for writing data (read-only)
	const byte BufferSize;
	//! Copies data from given byte array
	/*! Function checks for data integrity via CRC16 calcultation, then copies array
		\param input Received data from device
		\returns Data integrity checking result */
	bool SetBuffer(const byte* input);
	//! Generates CRC16 and copies data
	/*! \param buffer Where to copy data */
	void GetBuffer(byte* buff);
};
//! Class describes controller configuration endpoint
/*! Endpoint size: 24+2 bytes, acceptable addresses: get: 0x22, 0x32; set: 0x23, 0x33 */
class ControllerConfig : public SerialComm 
{
public:
	//! Class constructor
	/*! \param rxAddr Set endpoint address for reading data
		\param txAddr Set endpoint address for writing data */
	ControllerConfig(byte rxAddr, byte txAddr);
	//! Get minimum level for brightness regulation
	/*! \param channel Channel number, range [0;7]
		\returns Brightness level, range [0;255] */
	byte MinLvlGet(byte chnl);	
	//! Set minimum level for brightness regulation
	/*! \param channel Channel number, range [0;7]
		\param value Brightness level, range [0;255] */
	void MinLvlSet(byte chnl, byte value);
	//! Set brightness level directly
	/*! \param channels Array of channel numbers in range [0;8], length must be equal to 3
		\param val Brightness level, range [0;255] */
	void OverrideLvlSet(byte* channels, byte val);
	//! Get fade rate of link
	/*! \param link Link number, range [0;3]
		\returns Fade rate */
	byte FadeRateGet(byte link);
	//! Set fade rate of link
	/*! \param link Link number, range [0;3]
		\param value Fade rate */
	void FadeRateSet(byte link, byte value);
	//! Get delay of link
	/*! \param link Link number, range [0;3]
		\returns Delay */
	byte LinkDelayGet(byte link);
	//! Set delay of link
	/*! \param link Link number, range [0;3]
		\param value Delay */
	void LinkDelaySet(byte link, byte value);
	//! Set motion sensor low time
	/*! \param value Time in seconds, less than 6 means motion sensor is disabled */
	void MSenLowTimeSet(byte value);
	//! Get motion sensor low time
	/*! \returns Time in seconds */
	byte MSenLowTimeGet();
};

//! Class describes controller endpoint to set on time of one channel
/*! Endpoint size: 24+2 bytes, acceptable addresses: set: 0x23, 0x33 */
class ControllerOTSet : public SerialComm //24+2 bytes, set: 0x23, 0x33
{
public:
	//! Class constructor
	/*! \param rxAddr Set endpoint address for reading data
		\param txAddr Set endpoint address for writing data */
	ControllerOTSet(byte rxAddr, byte txAddr);
	//! Set channel on time and switch count
	/*! \param channel Channel number, range [0;8]
		\param onTime On time, in seconds
		\param swCnt On/off count */
	void ReplaceChOnTime(byte chnl, uint onTime, ushort swCnt);
};

//! Class describes controller state endpoint
/*! Endpoint size: 23+2 bytes, acceptable addresses: get: 0x21, 0x31 */
class ControllerState : public SerialComm
{
public:
	//! Class constructor
	/*! \param rxAddr Set endpoint address for reading data
		\param txAddr Set endpoint address for writing data */
	ControllerState(byte rxAddr, byte txAddr);
	//! Get controller on time
	/*! \returns Time in 1/32th of second */
	uint GetTicks();
	//! Get channel output level
	/*! \param channel Channel number, range [0;8]
		\returns Channel output level */
	byte GetChLevel(byte chnl);
	//! Get link output level
	/*! \param link Link number, range [0;3]
		\returns Link output level */
	byte GetLinkLevel(byte link);
	//! Determine if link input state is in acceptable range
	/*! \param link Link number, range [0;3]
		\returns True if link state is OK */
	bool IsLinkValid(byte link);
	//! Get state of motion sensor
	/*! \returns State of motion sensor */
	byte MSenStateGet();
};

//! Class describes controller endpoint to get on time of channels
/*! Endpoint size: 54+2 bytes, acceptable addresses: get: 0x24, 0x34 */
class ControllerChOnTime : public SerialComm
{
public:
	//! Class constructor
	/*! \param rxAddr Set endpoint address for reading data
		\param txAddr Set endpoint address for writing data */
	ControllerChOnTime(byte rxAddr, byte txAddr);
	//! Get channel on time
	/*! \param channel Channel number, range [0;8]
		\returns Channel on time in seconds */
	uint GetOnTime(byte chnl);
	//! Get channel switch count
	/*! \param channel Channel number, range [0;8]
		\returns Channel on/off cycles */
	ushort GetSwCount(byte chnl);
};

//! Class describes ventilation configuration endpoint
/*! Endpoint size: 9+2 bytes, acceptable addresses: get: 0x12, set: 0x13 */
class VentilationConfig : public SerialComm 
{
public:
	//! Class constructor
	/*! \param rxAddr Set endpoint address for reading data
		\param txAddr Set endpoint address for writing data */
	VentilationConfig(byte rxAddr, byte txAddr);
	//! Get fan level
	/*! \returns Fan level in range [0;100] */
	byte FanLevelGet();
	//! Set fan level
	/*! \param value Fan level, range [0;99]; 100 means Auto operation mode */
	void FanLevelSet(byte value);
	//! Get minimum RH level at which fan starts spinning
	/*! \returns Minimum RH level in range [50;80] */
	byte MinRHGet();
	//! Set minimum RH level at which fan starts spinning
	/*! \param value Fan level, range [50;80] */
	void MinRHSet(byte value);
};

//! Class describes ventilation configuration endpoint
/*! Endpoint size: 15+2 bytes, acceptable addresses: get: 0x11 */
class VentilationState : public SerialComm
{
public:
	//! Class constructor
	/*! \param rxAddr Set endpoint address for reading data
		\param txAddr Set endpoint address for writing data */
	VentilationState(byte rxAddr, byte txAddr);
	//! Get current fan level (useful in Auto operation mode)
	/*! \returns Fan level in range [0;100] */
	byte FanLevel();
	//! Get RPM of front impeller of ventilation fan
	/*! \returns Rotations per minute */
	ushort RPMFront();
	//! Get RPM of rear impeller of ventilation fan
	/*! \returns Rotations per minute */
	ushort RPMRear();
	//! Get current drawn by ventilation fan
	/*! \returns Current in mA */
	ushort CurrentDraw();
	//! Get RH
	/*! \returns RH in 0.1% */
	ushort GetRH();
	//! Get temperature
	/*! \returns Temperature in 0.1�C */
	short GetTemperature();
};

struct FloorEndpoints {
	ControllerConfig conf;
	ControllerState state;
	ControllerChOnTime chOTGet;
	ControllerOTSet chOTSet;
};
