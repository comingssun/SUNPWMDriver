#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
extern "C"{
#include<i2c/smbus.h>
}
#include <time.h>
#include <SUNPWMPCA9685.h>

// for pipe communication
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

// Debug mode
 #define DEBUG 1



// Calibrated for main BLDC motor
#define THROTTLE_FULL_REVERSE 1650//204 20ms 1241 // 1ms / 3.3ms * 4096
//#define PWM_NEUTRAL 307 //2048 // 1.5ms / 3.3ms * 4096
#define THROTTLE_NEUTRAL 2150 //2048 // 1.5ms / 3.3ms * 4096
#define THROTTLE_FULL_FORWARD 2300 //2482 // 2ms / 3.3ms * 4096
#define DIR_NEUTRAL 2050
//+- 500정도 움직임




#define SERVO_CHANNEL 10
#define ESC_CHANNEL 8

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value
// 체크
int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}

int main() {
    PCA9685 *pca9685 = new PCA9685();
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(300);
	sleep(1);
	pca9685->setPWM(ESC_CHANNEL, 0, THROTTLE_NEUTRAL);
	//pca9685->setPWM(SERVO_CHANNEL, 0, DIR_NEUTRAL);
#ifdef DEBUG
        // 27 is the ESC key
        printf("Hit 1 key to exit\n");
	int ACC = THROTTLE_NEUTRAL;

        while(pca9685->error >= 0 && getkey() != 27){

/*	if(ACC>=PWM_FULL_FORWARD)		//안전장치 설정
		ACC=PWM_FULL_FORWARD;
	else if(ACC<=PWM_FULL_REVERSE)
		ACC=PWM_FULL_REVERSE;
*/
	int i=0;
	    char mode,mode2;	
	    mode=getkey();
	if(mode=='r')
	{
		 while( mode != 'n'){
		mode=getkey();
		pca9685->setPWM(ESC_CHANNEL,0,THROTTLE_FULL_REVERSE); 
		printf("reverse\n");
		}
	}
	else if(mode=='n')
	{
	while(i<1000000){
	pca9685->setPWM(ESC_CHANNEL,0,THROTTLE_NEUTRAL); 
		printf("neutral\n");
		i++;}
	}
	else if(mode=='f')
	{
		while(mode!='r'){
		mode=getkey();
		pca9685->setPWM(ESC_CHANNEL,0,THROTTLE_FULL_FORWARD);
		printf("full\n");}
	}
	else if(mode=='1')
	goto exit;
/*
	    switch(mode){
		case 'r':
		   pca9685->setPWM(ESC_CHANNEL,0,THROTTLE_FULL_REVERSE); 
		printf("reverse\n");
		break;
		case 'n':
		  pca9685->setPWM(ESC_CHANNEL,0,THROTTLE_NEUTRAL); 
		printf("neutral\n");
		break;
		case 'f':
		  pca9685->setPWM(ESC_CHANNEL,0,THROTTLE_FULL_FORWARD);
		printf("full\n");
		break;
		case '1':
		  goto exit;
		}*/
	//printf("ACC :%d \n",ACC);
	sleep(1);
        }
	exit:
#endif
    pca9685->closePCA9685();
    }
}
