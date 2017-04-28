// this is code that I found on the web at 
// http://stackoverflow.com/questions/17254923/raspberry-pi-uart-program-in-c-using-termios-receives-garbage-rx-and-tx-are-con
// I have tested the code with the pincher ARM and I have gotten the TXD and the RXD to function 
// of course I have tailored to my needs 
// its so simple that I was very interested if this code actually worked 
// highly modified from the original code 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

static int Pwrite (int fd, char* buf, int numbytes);
static int Pread (int fd, char* buf, int numbytes);
int sendHomeCmd(void);
int sendReachCmd(void);
int sendGrabCmd(void);
int sendParkCmd(void);
int sendReleaseCmd(void);

// pincher arm command strings  
// this is an expansion of Joe Cabannas Pincher ARM Arduino code.  
// Joe had 9 commands that he used for demo purposes 
// So start our commands 10 
// Best to make these bytes and not use strings

#define Home         'a'
#define Reach        'b'
#define Grab         'c'
#define Park         'd'
#define Release      'e'
#define Victory      'f'

#define TRUE        0x01
#define FALSE       0x00

typedef enum
{
    STATUS_OK,
    STATUS_NACK
    
} _STATUS;
    
char buffer[10];
int fd, debug=TRUE ;

/*** \brief public routines 
 */ 
int setupPincherSerialPort(void);


/* start of routines */

/** \brief  setup the serial for the Pincher arm 
 * 
 * this routine is used to open the /dev/ttyUSB1 virtual com
 * port 
 *
 *
 */
 
int setupPincherSerialPort(void) {

    struct termios options;

    fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        perror("/dev/ttyUSB1 \n\r");
        return -1;
    }

#if 1
   fcntl (fd, F_SETFL, 0);
   tcflush (fd, TCIOFLUSH);
   //get config from fd and put into options
   //tcgetattr (fd, &options); 
   if (tcgetattr(fd, &options) < 0)
   {
        perror("Getting configuration");
        return -1;
    }
    //give raw data path
   cfmakeraw (&options);
   //set baud
   cfsetispeed (&options, B115200);
   cfsetospeed (&options, B115200);
    // Set up Serial Configuration
    //options.c_iflag = 0;
    //options.c_oflag = 0;
    //options.c_lflag = 0;
    //options.c_cflag = 0;
    options.c_cc[VMIN] = 1;
    //options.c_cc[VTIME] = 0;
    //serial.c_cflag = B115200 | CS8 | CREAD;
   //send options back to fd
   tcsetattr (fd, TCSANOW, &options);
#endif

}

#if 1 
/** \brief  send the home command to the Pincher 
 * 
 * write the home command to the Pincher arm 
 * places the arm in the home position
 *
 * problem - the arm is sending back a 0x30 not a 0x00. so the serial.print() is not correct
 *
 */
int sendHomeCmd(void)
{
    int i=0, numread;
    char cmd[1], buffer[5];
    
    cmd[0] = Home;
    
    if ( Pwrite(fd, cmd, sizeof(cmd) ) < 0 ) 
    {
      perror("Could not send the Home Command \n");
      return -1;
    }
#if 1    
    numread = Pread(fd, buffer, 1);  // the pincher will return 1 status byte
    if  (numread < 0)
    {
          perror ("No response from the Home Command \n");
         return -1;
    }

    if (buffer[0] == 0x30)
      return STATUS_OK;
    else
      return STATUS_NACK;  
#endif      
}

/** \brief  send the Reach command to the Pincher Arm
 * 
 * write the Reach command to the Pincher arm 
 * places the arm in the Reach position to grab the block 
 *
 * problem - the arm is sending back a 0x30 not a 0x00. so the serial.print() is not correct
 *
 */
int sendReachCmd(void)
{
    int i=0, numread;
    char cmd[1], buffer[5];
    
    cmd[0] = Reach;
       
    if ( Pwrite(fd, cmd, sizeof(cmd) ) < 0 ) 
    {
      perror("Could not send the Reach Command \n");
      return -1;
    }

#if 1  
    numread = Pread(fd, buffer, 1);  // the pincher will return 1 status byte
    if  (numread < 0)
    {
          perror ("No response from the Reach Command \n");
         return -1;
    }

    if (buffer[0] == 0x30)
      return STATUS_OK;
    else
      return STATUS_NACK;  
#endif      
}

/** \brief  send the Grab command to the Pincher Arm
 * 
 * write the Grab command to the Pincher arm 
 * grabs the block  
 *
 * problem - the arm is sending back a 0x30 not a 0x00. so the serial.print() is not correct
 *
 */
int sendGrabCmd(void)
{
    int numread;
    char cmd[1], buffer[5];
    
    cmd[0] = Grab;

    if ( Pwrite(fd, cmd, sizeof(cmd) ) < 0 ) 
    {
      perror("Could not send the Grab Command \n");
      return -1;
    }
    
    numread = Pread(fd, buffer, 1);  // the pincher will return 1 status byte
    if  (numread < 0)
    {
          perror ("No response from the Grab Command \n");
         return -1;
    }

    if (buffer[0] == 0x30)
      return STATUS_OK;
    else
      return STATUS_NACK;  
}

/*** \brief    send the Release command to the Pincher Arm
 * 
 * write the Release command to the Pincher arm 
 * Release the block  
 *
 * problem - the arm is sending back a 0x30 not a 0x00. so the serial.print() is not correct
 *
 */
int sendReleaseCmd(void)
{
    int numread;
    char cmd[1], buffer[5];
    
    cmd[0] = Release;
    if ( Pwrite(fd, cmd, sizeof(cmd) ) < 0 ) 
    {
      perror("Could not send the Release Command \n");
      return -1;
    }
    
    numread = Pread(fd, buffer, 1);  // the pincher will return 1 status byte
    if  (numread < 0)
    {
          perror ("No response from the Release Command \n");
         return -1;
    }

    if (buffer[0] == 0x30)
      return STATUS_OK;
    else
      return STATUS_NACK;  
}


/*** \brief     send the Park command to the Pincher Arm
 * 
 * write the Park command to the Pincher arm 
 * Park the ARM with the block in the gripper  
 *
 * problem - the arm is sending back a 0x30 not a 0x00. so the serial.print() is not correct
 *
 */
int sendParkCmd(void)
{
    int numread;
    char cmd[1], buffer[5];
    
    cmd[0] = Park;
    if ( Pwrite(fd, cmd, sizeof(cmd) ) < 0 ) 
    {
      perror("Could not send the Park Command \n");
      return -1;
    }
    
    numread = Pread(fd, buffer, 1);  // the pincher will return 1 status byte
    if  (numread < 0)
    {
          perror ("No response from the Park Command \n");
         return -1;
    }

    if (buffer[0] == 0x30)
      return STATUS_OK;
    else
      return STATUS_NACK;  
}





/*** \brief Write data to the Pincher's
 * 
 * Writes data to the Pincher's serial port.  This function will
 * continue to attempt writing to the serial port until the
 * specified number of bytes has been written or until nothing is
 * written 3 times in a row.  At that point, the function will
 * return the number of bytes written so far.  Use this function
 * to write to the Create instead of OS-specific functions.
 * 
 * \param   fd       The file descriptor for the serial port
 * \param   buf         The buffer to write from
 * \param   numbytes The number of bytes to write
 * 
 * \return     The number of bytes written to the port or -1 on error
 */
static int Pwrite (int fd, char* buf, int numbytes)
{
   int i, numwritten = 0, n = 0, numzeroes = 0;

   //write (fd, (buf + numwritten), (numbytes - numwritten));

   while (numwritten < numbytes)
   {
      n = write (fd, (buf + numwritten), (numbytes - numwritten));
      if (n < 0)
         return -1;
      if (0 == n)
      {
         numzeroes++;
         if (3 < numzeroes)
            break;
      }
      numwritten += n;
   }
      
   if (debug)
   {
      printf ("Write: ");
      for (i = 0; i < numwritten; i++)
         printf ("%d ", buf[i]);
      printf ("\nWrote %d of %d bytes\n", numwritten, numbytes);
   }
   
   return numwritten;
}

/** \brief Read data from the Pincher
 *
 * Reads data from the Pincher's serial port.  This function will
 * continue reading from the port until the specified number of
 * bytes has been read or until it reads nothing 3 times in a row
 * (this can happen if you tell the function to read more data
 * than the Create has available).  If this limit is reached, the
 * buffer will contain what was read up to that point and the
 * function will return the number of bytes read up to that
 * point.  Use this instead of the OS-specific read function when
 * reading from the Create.
 *
 * \param   fd       The file descriptor for serial port
 * \param   buf         The buffer to read the data into
 * \param   numbytes The number of bytes to read
 *
 * \return The number of bytes read from the serial port or -1 on
 * error
 */
static int Pread (int fd, char* buf, int numbytes)
{
   int i, numread = 0, n = 0, numzeroes = 0;
   
   while (numread < numbytes)
   {
      n = read (fd, (buf + numread), (numbytes - numread));
      if (n < 0)
         return -1;
      if (0 == n)
      {
         numzeroes++;
         if (3 < numzeroes)
            break;
      }
      numread += n;
   }
   
   if (debug)
   {
      printf ("Read:   ");
      for (i = 0; i < numread; i++)
         printf ("%d ", buf[i]);
      printf ("\nRead %d of %d bytes\n", numread, numbytes);
   }
       
   tcflush (fd, TCIFLUSH);       //discard data that was not read
   return numread;
}
#endif

#if 0 
int sendHomeCmd(void)
{
    int i=0, count;
    char cmd[1], ch;
    cmd[0] = Home;
    
    printf("Sending: %d\n", cmd[0]);

    if ((count = write(fd, cmd, sizeof(cmd))) < 0 ) 
    {
      perror("Could not send the Home command \n");
      return -1;
    }
   else 
        printf("Sent %d characters\n", count);
    // poll for the read data    
    while(1)
    {
        count = read(fd, &ch, 1);
        if  (count !=0)
        {
            buffer[i]=ch;
            i++;
            if(ch==0x30)
            {
                printf("Received %d characters\n", count);
                break;
            }
        }
        else if (count < 0)
        {
           perror ("Could not get Home cmd response \n");
         return -1;
        }
    }; // end while 1 

    if (buffer[0] == 0)
      return STATUS_OK;
    else
      return STATUS_NACK;  
}
#endif


// end of file 

