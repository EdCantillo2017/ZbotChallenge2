/** PincherApp.c
 *
 *  Team D ZBot Challenge #2
 *  -approach the block 1ft away
 *  -pick up the block
 *  -approach the target 4ft away
 *  -drop the block on the target
 *  -return home
 *  -all in less then 2 minutes
 * 
 *  Author: KK, MB and EC
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
 *	1.0 	12 Jan 2008	Initial public release
 */
#include <createoi.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include "SerialCommPincher.h"

/**
 * define 
 **/
#define ONE_FOOT_IN_MM  300
#define FOUR_FEET_IN_MM 1219.2
#define FIVE_FEET_IN_MM 1524

#define TRUE        1
#define FALSE       0
#define SKIP_INT		100
#define KP_RATIO		0.1
#define abs(x) ((x<0) ? -1 * x : x)
#define MASTER_DRV_SPEED	100
#define Inches2mm(x)	(x*25.4) 


/**
 *  local functions
 **/
static int MystopWait(void);

/** 
 * declarations
 **/
int distance1 = Inches2mm(13);  // punch in a distance in inches 
int distance2 = Inches2mm(44);  // punch in a distance in inches 
int distance3 = Inches2mm(62);  // punch in a distance in inches 
int interrupt = 1;
int iter=0;
int PID_enabled = TRUE;

/** \brief Main function 
 * 
 * ZBot challenge 2 control loop  
 * 
 * 
 */
int main(int argc, char* argv[]) {

    // initialize the virtual serial port
    startOI_MT ("/dev/ttyUSB0");
    setupPincherSerialPort(); // open the pincher com port
    sleep(10);	// allow the picher arm to initialize.
				// this really takes ~10 seconds 
				// but the execution of the Pincher tasks happen after the drive command   
	sendReachCmd();			
	travelDistance(MASTER_DRV_SPEED, distance1); // travel the desired as the first characterization testing 
    sendGrabCmd();
    sendParkCmd();
    travelDistance(MASTER_DRV_SPEED, distance2); // travel the desired as the first characterization testing 
    sendReachCmd();
    sendReleaseCmd();
    sendHomeCmd();
    travelDistance(-500, distance3); // travel the desired as the first characterization testing 

    stopOI_MT();  // close the comm ports 
    close(fd);
}



static int MystopWait(void)
{
    int shouldStop=0;
    
	shouldStop += getBumpsAndWheelDrops() + getCliffs();
  
    return shouldStop;
}


/** \brief travel the set speed for the set distance 
 * 
 * 	uses the encoder counts to determine the distance 
 * 	traveled. 
 *  uses the direct drive command to drive the motors  
 * 
 * 	\param Speed is in mm/sec. postive values moves forward  
 *  \param Distance is in mm
 * 
 *	added drift compensation by using the delta from the 
 *  left and right encoder counts to change speed of the slaveDrv 
 * 
 * 
 **/
int travelDistance(int Speed, int distance)
{
    
   	// start of the distance traveled routine 
    float fact;  // trial and error
    int slaveDrv, current, error, previous_err, totalDist = 0, adjust=0;
    directDrive(Speed,slaveDrv=Speed); // start both at the same speed 
    sleep(1);

    while(1)
    {
		current = getDistance(); 
		if (INT_MIN == current)
			return INT_MIN;
		totalDist += current;
		printf("waitCurrent = %d & waitTotalDist = %d \r\n", current, totalDist);
		if ((interrupt && MystopWait()) 
		    || (distance >= 0 && totalDist >= distance) 
		    || (distance < 0 && totalDist <= distance))
         		break;
		
        if ( PID_enabled) 
        {
            error = getLeftEncoder() - getRightEncoder(); // get the error from the encoders 
            if ( error < 0 )
            {   // the right is faster
                // subract from the right and add to left 
                adjust = error/2;
                printf("adjust < 0 %d \r\n", adjust);
                if(adjust < -20 )
                    adjust = -20;
                directDrive(Speed+adjust, slaveDrv-adjust);  // left drive is the master drive 
            }
            if ( error > 0)
            { // the left is faster 
                // subract from left and add to right 
                adjust = error / 2;
                if (adjust > 20 )
                    adjust = 20;
                printf("adjust > 0 %d \r\n", adjust);
                directDrive(Speed-adjust, slaveDrv+adjust);  // left drive is the master drive 
            }
            
                        
#if 0            
            if (error != previous_err)
            {
            // PID controller but I am doing just Proportional 
            // keep it simple stupid: apply KISS
            // trial and error to determine the Kp value 

                previous_err = error;
                fact = error * KP_RATIO; //should be between 0 and 1. TODO: interger math 
                slaveDrv += fact;
                directDrive(Speed, slaveDrv);  // left drive is the master drive 
            }
#endif            
        }
        
		usleep(60000);  // wait for the sensor to update 
				       
    }
    directDrive(0,0);
    sleep(1);
    return 0;
}



