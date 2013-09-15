
/* P3 is connected to the 8 GPIOs dedicated to the Polulu QTR-8RC sensor array
** pins l1-l8 refer to the 8 GPIOs connected to the 8 LEDs that display the status of the sesnor array
** pin s_ON is connected to the LEDON pin of the sensor array*/

#include <reg51.h>
#include <reg_c51.h>

sbit s1=P2^0;
sbit s2=P2^1;
sbit s3=P2^2;
sbit s4=P2^3;
sbit s5=P2^4;
sbit s6=P2^5;
sbit s7=P2^6;
sbit s8=P2^7;


sbit cal=P3^3; //this can be any one of the pins of P3 (sensor array), is used for calibration
sbit buzzer_pin=P0^7; //connected to a Vcc pin of a small 5V buzzer (small enough to be driven by a GPIO's o/p current)
sbit s_ON=P0^5;

void readSensor();
void delay(long int);
void beep(long int); 
void longBeep(char);
void supercalib();
long int calib();

long int whiteDelay;

													   	
void main()
{ 	
	
	supercalib(); //calibrate the value of 'white delay' - the amount of time required to read a white patch.
		//If sensor does not return high in the time 'whiteDelay', then the patch being read has to be black.
	
	while(1)
	{
		readSensor(); //take a sensor reading. These values are now stored in s1, s2.. s8.
		
		/*
		
		...
		
		Insert lines of code that use the sensor readings - which can be bit-accessed in s1, s2, s3... s8.
		For example, 
		
		if(s1==0&&s2==0&&s7==0&&s8==0) moveForward();
		
		*/
		
	}
}
	
void readSensor()
{
    	s_ON=1; // switch on the entire sensor arrray

	delay(10); 
	
	P3=0xFF; //set Port 3 as output
	P3=0xFF; //set all sensor pins as high
	
	smallDelay(whiteDelay); //wait for the specific amount of time required to read white. 
	
	P2=P3; //assign the readings to Port 2 i.e. the LEDs.
	
	s_ON=0; //switch off the entire sensor array

	delay(10);
}

long int calib()
{
	
	long int sens;
	sens=0;
    	
    	s_ON=1; // switch on the entire sensor arrray
	
	P3=0xFF; //set Port 3 as output
	P3=0xFF; //set all sensor pins as high
	
	while(cal==1) //as long as the 'cal' sensor pin is high, keep incrementing 'sens'
	{
		sens++;
		if(sens>10000)break;
	}
	
	s_ON=0; //switch off the entire sensor array
	
	delay(10);
	
	return sens;
}

void supercalib()
{
	/*to start with, put the STOP pin to ground*/
	int blackDelay;

	longBeep(1);
	longBeep(1); //Place all the sensors on a completely white area
	delay(40000); 
	longBeep(1); //starting white calibration!

	whiteDelay=calib();
	if(whiteDelay<=0)whiteDelay=30;

	delay(10000);//Place all the sensors on a completely black area
	longBeep(2);
	delay(40000);
	longBeep(2); //starting black calibration!
	
	blackDelay=calib();
	delay(40000);
	longBeep(3);
	delay(10000);
	
	if(blackDelay<whiteDelay)
	{
		whiteDelay=1200;
	}
	
	/*The below commented code allows you to hear a number of beeps proportional to the calibrated values, 
	to check how well the calibration has worked. Uncomment it if you'd like that*/
	
	/*beep((blackDelay-whiteDelay)/300);
	
	delay(10000);
	longBeep(4);
	delay(10000);
	
	beep(blackDelay/300);
	delay(10000);
	longBeep(5);
	delay(10000);
	
	beep(whiteDelay/300);*/
	
	while(STOP==1); //the program will wait for you to set the STOP pin to low and then begin motion.
}

void delay(long int i)
{
    long int j;
    for(j=0;j<i;j++)
    {
        j+=1;
        j-=1;
    }
}


void beep(long int c)
{
	long int i;
	
	for(i=0;i<c;i++)
	{
		buzzer_pin=0;
		delay(1000);
		buzzer_pin=1;
		delay(1000);
	}
}


void longBeep(unsigned char c)
{
	unsigned char i;
	
	for(i=0;i<c;i++)
	{
		buzzer_pin=0;
		delay(3000);
		buzzer_pin=1;
		delay(3000);
	}
}
