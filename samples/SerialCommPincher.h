/*** Serial comm pincher header file  
*
*	
*
*
*
*/

extern int setupPincherSerialPort(void);
extern int sendHomeCmd(void);
extern int sendReachCmd(void);
extern int sendGrabCmd(void);
extern int sendParkCmd(void);
extern int sendReleaseCmd(void);
extern int fd; // pincher file descriptor. lame name 