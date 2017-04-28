/** \file createoi.h
 *  \brief Header file for createoi.c.
 *
 *  Header file for createoi.c and for COIL.  #include this file in
 *  your program to make use of COIL functions.  This file contains
 *  signatures for the available CreateOI functions as well as
 *  definitions for the various datatypes used for them.  Info on the
 *  different types can be found in the included documentation.
 *
 *  \authors	Jesse DeGuire, Nathan Sprague
 *		Andrew C. Smith
 *
 * 
 *  This file is part of COIL.
 *
 *  COIL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  COIL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * 
 *  You should have received a copy of the GNU General Public License
 *  along with COIL.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *
 *  Versions:
 *	1.0	12 Jan 2008	Initial public release
 *      1.1     04 Aug 2008     Added multi-threaded mode. 
 *	2.0	14 Apr 2015	Updated for Create 2.
 */

#ifndef H_CREATEOI_GD
#define H_CREATEOI_GD

#include <unistd.h>


#ifdef __cplusplus
extern "C" {
#endif

//
#define TICK_PER_REV        508.8
#define WHEEL_DIAMETER      72
#define WHEEL_BASE          235
#define TICK_TO_DISTANCE    0.44456499814949904317867595046408

/// 8-bit unsigned value.  Called "byte" to keep continuity with iRobot naming
/// (and so I don't have to write "unsigned char" all the time).
typedef unsigned char	byte;


/** \brief Command Opcodes
 *
 *  Defines constants for all of the available opcodes for the Create OI.
 */
typedef enum
{
	OPCODE_START				= 128,
	OPCODE_BAUD,
	OPCODE_CONTROL,
	OPCODE_SAFE					= 131,
	OPCODE_FULL,
	OPCODE_POWER,
	OPCODE_SPOT					= 134,
	OPCODE_CLEAN,
	OPCODE_MAX_CLEAN,
	OPCODE_DRIVE,
	OPCODE_MOTORS,
	OPCODE_LED,
	OPCODE_SONG,
	OPCODE_PLAY_SONG,
	OPCODE_QUERY,
	OPCODE_FORCE_DOCK,
	OPCODE_PWM_MOTORS,
	OPCODE_DRIVE_WHEELS,
	OPCODE_DRIVE_PWM,
	OPCODE_STREAM				= 148,
	OPCODE_QUERY_LIST,
	OPCODE_PAUSE_RESUME_STREAM,
	OPCODE_SCHEDULE_LED			= 162,
	OPCODE_DIGIT_LED_RAW,
	OPCODE_DIGIT_LED_ASCII,
	OPCODE_BUTTONS,
	OPCODE_SCHEDULE				= 167,
	OPCODE_SET_DAY_TIME,
	OPCODE_STOP				= 173
} oi_command;

/** \brief Sensor packet constants
 *
 *  Contains constants for each of the 42 sensor packets available in
 *  the Create.  Note that not all packets are the same size.  Refer
 *  to the Create Open Interface documentation for more info.
 */

#define SENSOR_GROUP_0_SIZE         26
#define SENSOR_GROUP_1_SIZE         10
#define SENSOR_GROUP_2_SIZE         6
#define SENSOR_GROUP_3_SIZE         10
#define SENSOR_GROUP_4_SIZE         14
#define SENSOR_GROUP_5_SIZE         12
#define SENSOR_GROUP_ALL_SIZE       52
#define SENSOR_GROUP_100_SIZE       80
#define SENSOR_GRUOP_101_SIZE       28
#define SENSOR_GROUP_106_SIZE       12
#define SENSOR_GROUP_107_SIZE       9

typedef enum
{
	SENSOR_GROUP_0			 	= 0,
	SENSOR_GROUP_1,
	SENSOR_GROUP_2,
	SENSOR_GROUP_3,
	SENSOR_GROUP_4,
	SENSOR_GROUP_5,
	SENSOR_GROUP_ALL,
	SENSOR_GROUP_100			= 100,
	SENSOR_GRUOP_101,
	SENSOR_GROUP_106			= 106,
	SENSOR_GROUP_107,
	SENSOR_BUMPS_AND_WHEEL_DROPS		= 7,
	SENSOR_WALL,
	SENSOR_CLIFF_LEFT,
	SENSOR_CLIFF_FRONT_LEFT,
	SENSOR_CLIFF_FRONT_RIGHT,
	SENSOR_CLIFF_RIGHT,
	SENSOR_VIRTUAL_WALL,
	SENSOR_OVERCURRENT,
	SENSOR_DIRT_DETECT,
	SENSOR_INFRARED			= 17,
	SENSOR_BUTTONS,
	SENSOR_DISTANCE,
	SENSOR_ANGLE,
	SENSOR_CHARGING_STATE,
	SENSOR_VOLTAGE,
	SENSOR_CURRENT,
	SENSOR_BATTERY_TEMP,
	SENSOR_BATTERY_CHARGE,
	SENSOR_BATTERY_CAPACITY,
	SENSOR_WALL_SIGNAL,
	SENSOR_CLIFF_LEFT_SIGNAL,
	SENSOR_CLIFF_FRONT_LEFT_SIGNAL,
	SENSOR_CLIFF_FRONT_RIGHT_SIGNAL,
	SENSOR_CLIFF_RIGHT_SIGNAL,
	SENSOR_CHARGING_SOURCES_AVAILABLE	 = 34,
	SENSOR_OI_MODE,
	SENSOR_SONG_NUMBER,
	SENSOR_SONG_IS_PLAYING,
	SENSOR_NUM_STREAM_PACKETS,
	SENSOR_REQUESTED_VELOCITY,
	SENSOR_REQUESTED_RADIUS,
	SENSOR_REQUESTED_RIGHT_VEL,
	SENSOR_REQUESTED_LEFT_VEL,
	SENSOR_ENCODER_COUNTS_LEFT,
	SENSOR_ENCODER_COUNTS_RIGHT,
	SENSOR_LIGHT_BUMPER,
	SENSOR_LIGHT_BUMP_LEFT,
	SENSOR_LIGHT_BUMP_FRONT_LEFT,
	SENSOR_LIGHT_BUMP_CENTER_LEFT,
	SENSOR_LIGHT_BUMP_CENTER_RIGHT,
	SENSOR_LIGHT_BUMP_FRONT_RIGHT,
	SENSOR_LIGHT_BUMP_RIGHT,
	SENSOR_IR_OPCODE_LEFT,
	SENSOR_IR_OPCODE_RIGHT,
	SENSOR_LEFT_MOTOR_CURRENT,
	SENSOR_RIGHT_MOTOR_CURRENT,
	SENSOR_MAIN_BRUSH_CURRENT,
	SENSOR_SIDE_BRUSH_CURRENT,
	SENSOR_STASIS
} oi_sensor;

/** \brief  Baud Rate Codes
 *
 *  Codes to set the baud rate, in bits per second, at which data is
 *  sent over the serial port.  Use with the Baud command.
 */
typedef enum
{
	BAUD300,
	BAUD600,
	BAUD1200,
	BAUD2400,
	BAUD4800,
	BAUD9600,
	BAUD14400,
	BAUD19200,
	BAUD28800,
	BAUD38400,
	BAUD57600,
	BAUD115200
} oi_baud;

/** \brief LED switches
 *
 *  Used for turning off and on the LEDs on top of the Create.  Use
 *  with the LEDs command.
 */
typedef enum
{
	LED_CHECK_ROBOT		= (1<<3),
	LED_DOCK		    = (1<<2),
	LED_SPOT		    = (1<<1),
	LED_DEBRIS		    = (1<<0)
} oi_led;

/** \brief Output switches
 *
 *  Used for setting the the state of the motor outputs in OPCODE_MOTORS (138)
 */
typedef enum
{
	SIDE_BRUSH	    = 1,
	VACUUM		    = 2,
	MAIN_BRUSH	    = 4,
	SIDE_REVERSE	= 8,
	MAIN_REVERSE	= 16
} oi_output;

int startOI (const char* serial);
int startOI_MT (const char* serial);
int setBaud (oi_baud rate);
int enterSafeMode ();
int resetCmd(void);
int enterFullMode ();
int powerOff ();
int runSpot ();
int runClean ();
int runMaxClean ();
int runForceDock ();
int drive (short vel, short rad);
int directDrive (short Lwheel, short Rwheel);
int driveDistance (short vel, short rad, int dist, int interrupt);
int turn (short vel, short rad, int angle, int interrupt);
int setLEDState (oi_led lflags, byte pColor, byte pInten);
int setPWMLowSideDrivers (byte pwm0, byte pwm1, byte pwm2);
int setLowSideDrivers (oi_output oflags);
int sendIRbyte (byte irbyte);
int writeSong (byte number, byte length, byte* song);
int playSong (byte number);
int readRawSensor (oi_sensor packet, byte* buffer, int size);
int readSensor (oi_sensor packet);
int getDistance ();
int getAngle ();
int getVelocity ();
int getTurningRadius ();
int getOvercurrent ();
int getBumpsAndWheelDrops ();
int getCliffs ();
int getRightEncoder(void);
int getLeftEncoder(void);
int* getAllSensors ();
int readRawSensorList (oi_sensor* packet_list, byte num_packets, byte* buffer, int size);
double waitTime (double time);
int waitDistance (int dist, int interrupt);
int waitAngle (int angle, int interrupt);
int stopOI ();
int stopOI_MT ();
void enableDebug ();
void disableDebug ();

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif



#endif //H_CREATEOI_GD
