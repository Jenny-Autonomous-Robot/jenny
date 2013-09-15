/*

Complete program for Jenny's participation in APOGEE 2013 - Trackomania Robotics Competition
Problem statement URL: http://www.bits-apogee.org/2013/Automation/Track-o-Mania/

----------------------------------------------------------------
Written by : Pranav N. Gour, Arun Subramaniyan, Sujay Narumanchi
---------------------------------------------------------------


We are yet to add comments for individual methods. In the meanwhile, we would be
glad to repsond to any specific queries via email:

Pranav : f2011075@pilani.bits-pilani.ac.in
Arun : f2011212@pilani.bits-pilani.ac.in
Sujay : f2011025@pilani.bits-pilani.ac.in


************************************************************************

Hardware Interfacing required:

P3 is connected to the 8 GPIOs dedicated to the Polulu QTR-8RC sensor array
pins l1-l8 refer to the 8 GPIOs connected to the 8 LEDs that display the status of the sesnor array
pin s_ON is connected to the LEDON pin of the sensor array
Motor Driver control pins of left and right motors are connected to mb1, mb2, ma1, ma2.
pin deadend_ir : IR transmitter-receiver pair's o/p pin for sensing proximity to objects in front
sold_pin : IR transmitter-receiver pair's o/p pin for sideways object detection
terr_pin : IR transmitter-receiver pair's o/p pin for sideways object detection at a different height than sold_pin
buzzer_pin : output to buzzer
tagsMatter : swicth to control tag's status in the program
servo_pin : connected to servo controlling the gate to drop packages
s_on : Connected to LEDON pin of Polulu QTR-8RC sensor array
STOP : switch, starts program execution after initial calibration
*/

#include <reg51.h>
#include <reg_c51.h>

sbit mb1=P1^2;
sbit mb2=P1^5;
sbit ma1=P1^6;
sbit ma2=P1^7;


sbit s1=P2^0;
sbit s2=P2^1;
sbit s3=P2^2;
sbit s4=P2^3;
sbit s5=P2^4;
sbit s6=P2^5;
sbit s7=P2^6;
sbit s8=P2^7;

sbit cal=P3^3;

sbit deadend_ir=P0^0;

//sbit disp_sold = P0^1;
//sbit disp_terr = P0^2;

sbit sold_pin = P0^1;
sbit terr_pin = P0^2;

sbit buzzer_pin=P0^7; //Vcc pin of small 5V buzzer

sbit tagsMatter = P1^0; //control pin - can be a switch

sbit servo_pin=P1^1;//gate-controlling servo to drop package
		   

sbit s_ON=P0^5;

sbit STOP=P0^3;

//void forward();
//void backward();
//void turnLeft();
//void spinLeft();
//void turnRight();
//void spinRight();
//void testPWM();
void stopNow();
void smallDelay(long int);
void setInitial();
void readSensor();
void delay(long int);
int assignValue();
void assignPID();
void setMotors(int,int);
void decisionForward();
char checkLeftTag();
char checkRightTag();
char doubleCheckDeadEnd();
void disp();
void delayFlash(long int);
void beep(long int); 
void longBeep(char);
void servoControl(int pos);
void detectSoldier();
void detectTerrorist();
void testServos();
void moveGate(int);
void openGate();
void closeGate();
void supercalib();
long int calib();
char count_sold,count_terr;
char xleft;
char yleft;
long int whiteDelay;
int kp;
int MAX_SPEED;
int result;
int tag;
int sensor_delay=200;
long int delay_count=0;
tag=0; 
													   	
void main()
{ 	
	int lastTurn;
	setInitial();
	STOP=1;
	
	supercalib();

	closeGate();  //close gate
	setMotors(100,100);
	delay(1000);

	if(tagsMatter==1) MAX_SPEED=200;
	else MAX_SPEED=160;
	

	while(1)
	{
		
		//testServos();
		
		if(STOP==1)
		{
			stopNow();
			disp();
			while(1);
		}
		
		readSensor();
		result = assignValue();

		 if(deadend_ir==1) // dead end case
		 {	
		 	
		 	 if(doubleCheckDeadEnd()== 0) goto notDeadEnd;
			 
			 longBeep(1);
			 
			 if(tag==101)setMotors(-100,100); 
			 else if(tag==201) setMotors(100,-100);
			 else if(lastTurn>=45) setMotors(-100,100);
			 else setMotors(100,-100);
			 
			 while(1)
			 {
				 readSensor();
				 if((s2==0)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==0))break;
			 }
			 
			 while(1)
			 {
			 	readSensor();
				if(s4==1||s5==1) 
				{
					stopNow();
					break;
				}
			 }
			 
			//lemme explain tag. to start with, tag=0; tag=100 when tag to the right detected, =101 when turn has been taken,
//				 =102 if after this a dead end is encountered and we have completed u turn and found the line again.
			 //same for left. when doen with this, tag =0 again.
			if(tag==101) 
			{
				tag=102; 
				//beep(1); //beep is just for debugging
			}
			 
			else if(tag==201)
			{
				tag=202; 
			//	beep(2);
				
			}

			setMotors(-80,-80);
			delay(3000);
			stopNow();
			setMotors(100,100);
		}	
notDeadEnd:		
		if ((tagsMatter==1)&&(tag == 100) && ((((s6==1)&&(s5==1))&&((s4==1&&s3==1)||(s4==1&&s7==1)||(s7==1&&s8==1))) || ((s3==1)&&(s6==1)&&((s2==1&&s5==1)||(s4==1&&s7==1)||(s2==1&&s7==1))))  ) //@@@WORK ON THIS
		{ //if right tag is recorded and an intersection / Y fork is encountered	
			MAX_SPEED=200;
			decisionForward();	 
			readSensor();
			 if(s4==1||s5==1)
			 {
			 	//setMotors(70,70);
			//	delay(5000);
			 	setMotors(70,0);
			 	while(1)
				{
					readSensor();
					if(s4==0&&s5==0) break;
				}
			 }
				 	
			 setMotors(70,-70);
			 while(1)
			 {
				 if((s2==0)&&((s4==1)||(s5==1))&&(s7==0))break;	 						 
				 readSensor();
			 }
			 setMotors(70,70);
			 tag=101;
			 beep(3);
		}	
					
		else if ((tag==102) && (s3==1)&&(s4==1)&&(s5==1)&&(s6==1)) //if right tagged turn was taken, dead end encountered and now we're returning from u turn, and we encounter our fork/intersection.
		{
			decisionForward();
		 if(s4==1||s5==1)
		 {
			setMotors(100,100);
			tag = 0;
			delay(3000);
			beep(3);
			continue;
		 }
		 setMotors(100,-100);
		 	while(1)
			 {	
				readSensor();
				 
				 if((s3==0)&&(s4==0)&&(s5==0)) //TO DO : replace this with a delay or some other function, because this is good for T's but not for Y's
				 {
					 
					 break;
				    
				}
			 }
			setMotors(100,-100);
			while(1)
			 {
				 readSensor();
				 if((s2==0)&&((s3==1)||(s4==1)||(s5==1)||(s6==1))&&(s7==0))break;
			 }
			tag=0;
			setMotors(100,100);
			beep(3);
		 }
		 else if((tag==102)&&(s2==1)&&(s3==1)&&(s4==1)) // if returning from right tagge turn's dead end, and pure leeft turn encountered
		 {
			setMotors(100,100);
			tag = 0;
			delay(5000);
			beep(3);
			//stopNow();
			//readSensor();
			//result=assignValue();

			//setMotors(100,100);
			
		 }
				
		else if ((tagsMatter==1)&&(tag == 200) && ((((s3==1)&&(s4==1))&&((s5==1&&s6==1)||(s5==1&&s2==1)||(s2==1&&s1==1))) || ((s3==1)&&(s6==1)&&((s2==1&&s5==1)||(s4==1&&s7==1)||(s2==1&&s7==1))))  ) //@@@WORK ON THIS
		{ //if right tag is recorded and an intersection / Y fork is encountered
			//beep(1);
			MAX_SPEED=200;
			decisionForward();
				 
			readSensor();
			 if(s4==1||s5==1)
			 {
			 	//setMotors(70,70);
			//	delay(5000);
			 	setMotors(0,70);
			 	while(1)
				{
					readSensor();
					if(s4==0&&s5==0) break;
				}
			 }
			 	
			 setMotors(-70,70);
			 while(1)
			 {
				 if((s2==0)&&((s4==1)||(s5==1))&&(s7==0))break;	 						 
				 readSensor();
			 }
			 setMotors(70,70);
			 tag=201;
			 beep(2);
		}
			 			 
	 	 else if( (tag==202) && (s3==1)&&(s4==1)&&(s5==1)&&(s6==1))
		 {
		 decisionForward();
		 if(s4==1||s5==1)
		 {
			setMotors(100,100);
			tag = 0;
			delay(5000);
			beep(2);
			continue;
		 }
		 setMotors(-100,100);
		 while(1)
		 {	
				readSensor();
			 
			 if((s4==0)&&(s5==0)&&(s6==0))
			 {
				 
				 break;
			    
			}
		  }
			setMotors(-100,100);
			while(1)
			 {
				 
				 if((s2==0)&&((s3==1)||(s4==1)||(s5==1)||(s6==1))&&(s7==0))break;
				 						 
				 readSensor();
				 
			 }
			tag=0;
			setMotors(100,100);
			beep(2);
		 }
	     else if ((tag==202)&&(s5==1)&&(s6==1)&&(s7==1)&&(s8==1))
	     {
				setMotors(100,100);
				tag = 0;
				delay(5000);
				beep(2);
				/*setMotors(100,100);
				delay(2000);
				stopNow();
				readSensor();
				result=assignValue();
				if(result==0)
				{	setMotors(100,0);
				 while(1)
				 {
						if((s1==0)&&(s2==0)&&((s3==1)||(s4==1)||(s5==1)||(s6==1))&&(s7==0)&&(s8==0))break;
				 }
			  }
				else
				{
					setMotors(100,100);
					tag = 0;
				}*/
		 }  
			 
		 else if(result==0)
		{
			if(lastTurn<45) setMotors(-70,70);
			else if(lastTurn>=45)setMotors(70,-70);
			while(result==0)
			{ 
				readSensor();
				result = assignValue();
			}
		}
		else if(result==1111)//tag can be 0, 101, 201 for normal intersection with no tag
		{		
			setMotors(70,70);
			decisionForward();	//@@@MAKE THIS SENSOR BASED NOT TIME BASED..
		    readSensor();
     	    result = assignValue();
			if(result==0)
			{
				setMotors(70,0);
				while(result==0)
				{
					 readSensor();
					 result=assignValue();
				}
					 //beep(10);					 
					 //setMotors(70,70);
			}
		} 
		else
		{
			lastTurn=result;
			assignPID();
		}
		
		detectSoldier();
		detectTerrorist();	
	}
}
void detectSoldier()
{
	if((xleft==0)&&(sold_pin==1)&&(terr_pin==1))
	{
		stopNow();
		delay(1000);
		if((sold_pin==1)&&(terr_pin==1))
		{
			xleft=1;
			count_sold++;
			longBeep(count_sold);
			if(count_sold==3) openGate();
		}
	}	
	else if( (xleft==1)&&(sold_pin==0)&&(terr_pin==0))
	{
		stopNow();
		delay(1000);
		if( (sold_pin==0) && (terr_pin==0) )
		{
			xleft=0;
		}
	}
}
void detectTerrorist()
{
	if((yleft==0)&&(terr_pin==1)&&(sold_pin==0))
	{
		stopNow();
		delay(1000);
		if((yleft==0)&&(terr_pin==1)&&(sold_pin==0))
		{
			yleft=1;
			count_terr++;
			beep(count_terr);
		}
	}	
	else if( (yleft==1)&&(terr_pin==0)&&(sold_pin==0))
	{
		stopNow();
		delay(1000);
		if( (yleft==1)&&(terr_pin==0)&&(sold_pin==0))
		{
			yleft=0;
		}
	}
} 

char doubleCheckDeadEnd()
{
	stopNow();
	if(deadend_ir==0) return 0;

	setMotors(40,40);
	delay(2000);
	stopNow();
	if(deadend_ir==0) return 0;
	setMotors(-40,-40);
	delay(1000);
	stopNow();
	if(deadend_ir==0) return 0;

	return 1;
}

void decisionForward()
{
	stopNow();
	setMotors(70,70);
	readSensor();
	result=assignValue();
	delay_count=0;
	while((delay_count<30) && (result!=0))
	{
		delay(10);
		delay_count++;
		readSensor();
		result=assignValue();
	}
	stopNow();
	delay_count=0;
	return;
} 

	
int assignValue()
{
	int den=0;
	int res=0;

	if((s1==0)&&(s2==0)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==0)&&(s8==0))
	{
		return 0;//to check if it is completely white and hence take the turn previously taken
	}

	else if((s2==1)&&(s3==1)&&(s4==1)&&(s5==1)&&(s6==1)&&(s7==1))
	{
		return 1111;//to check if all the sensors are black and if true move forward and check
		
	}
	
	//else if ((((s5==1)||(s4==1))&&(((s7==0)&&(s8==1))||((s6==0)&&(s7==1)))))
	else if(checkLeftTag()==1)
	{
		if(((tag!=0)&&(tag!=101)&&(tag!=201))||(tagsMatter==0)) return 45; 	
		setMotors(-100,-100);
		delay(1400);
		stopNow();

		beep(2);
		
		delay(20000);

		readSensor();
		if(checkLeftTag()==0) return 45;

		delay(1000);
		if(checkLeftTag()==0) return 45;

		tag = 200;
		
		beep(4);
		MAX_SPEED=220;
		return 45;
	}

	else if(checkRightTag()==1)
	{
		if(((tag!=0)&&(tag!=101)&&(tag!=201))||(tagsMatter==0)) return 45; 	
		setMotors(-100,-100);
		delay(1400);
		stopNow();

		beep(3);
		delay(20000);
		readSensor();
		if(checkRightTag()==0) return 45;
		
		delay(1000);
		if(checkRightTag()==0) return 45;

		tag = 100;
		
		beep(5);
		MAX_SPEED=220;
		return 45;
	}

	else // IMP : this is where to implement PID
	{		
			den=(int)s2+(int)s3+(int)s4+(int)s5+(int)s6+(int)s7;
			if(den==0) //for when it's lost the line and is trying to find it.
			{
				if(s1==1) return 20;
				else return 70;
			}
			res=((((int)s2*20)+((int)s3*30)+((int)s4*40)+((int)s5*50)+((int)s6*60)+((int)s7*70))/den);
	
			return res;
	}
}


char checkRightTag()
{	
	if( ( ((s4==1)||(s5==1))&&((s6==0)||(s7==0))&&(s8==1) ) || ( (s4==1)&&((s5==0)||(s6==0))&&(s7==1)) )	
	return 1;

	else 
	return 0;
}

char checkLeftTag()
{
	if (((s1==1)&&((s2==0)||(s3==0))&&((s4==1)||(s5==1))) || ((s2==1)&&((s3==0)||(s4==0))&&(s5==1)))
	return 1;
	
	else
	return 0;
}

void assignPID()
{
	int leftval,rightval;
	int prop,pow_diff;
	prop=result-45;

	pow_diff=kp*prop;

	if(pow_diff>0)
	{
		leftval=100;
		rightval=100-pow_diff;
		if(rightval<-100) rightval=-100;
	}
	else
	{
		rightval=100;
		leftval=100+pow_diff;
		if(leftval<-100) leftval=-100;
	}
	
	
	setMotors(leftval,rightval);

}

void setMotors(int lval, int rval)
{
	if(lval>=0)
	{
		lval=(((MAX_SPEED-255.0)*lval)/100.0) + 255.0;
		ma1=0;
		ma2=1;
	}
	else
	{
		lval=(((MAX_SPEED-255.0)*((-1)*lval))/100.0) + 255.0;
		ma1=1;
		ma2=0;
	}
	if(rval>=0)
	{
		rval=(((MAX_SPEED-255.0)*rval)/100.0) + 255.0;
		mb1=0;
		mb2=1;
	}
	else
	{
		rval=(((MAX_SPEED-255.0)*((-1)*rval))/100.0) + 255.0;		
		mb1=1;
		mb2=0;
	}
	CCAP0L=(char)rval;
	CCAP0H=(char)rval;
	CCAP1L=(char)lval;
	CCAP1H=(char)lval;
}

void setInitial()
{
	stopNow();
		
	kp=5;

	buzzer_pin=1;

    IE=0x82;
    TCON=0x00;
    
    TMOD=0x12; // for sensor reading- using timer 1 in mode 1 ; for pwm clock-using timer 0 in mode 2
    
    TR1=0;
    TF1=0;
    
    TH0=0x00;
    TR0=0;
    TF0=0;
    
    TR0=1;
    
    CMOD=0x04; //for pwm to motor, using pca timer
    CL=0x00;
    CH=0x00;
    CCAPM0=0x42;
    CCAP0L=0xD3;
    CCAP0H=0xD3;
    CCAPM1=0x42;
    CCAP1L=0xD3;
    CCAP1H=0xD3;
    CCON=0x40;

}
 /*   
/*void forward()
{
    ma1=0;
    ma2=1;
    mb1=0;
    mb2=1;
}*
*/
void stopNow()
{
    ma1=0;
    ma2=0;
    mb1=0;
    mb2=0;
}

/*void backward()
{
    ma1=1;
    ma2=0;
    mb1=1;
    mb2=0;
}

void turnRight()
{
    ma1=0;
    ma2=0;
    mb1=0;
    mb2=1;
}
void spinRight()
{
    ma1=1;
    ma2=0;
    mb1=0;
    mb2=1;
}
void turnLeft()
{
    ma1=0;
    ma2=1;
    mb1=0;
    mb2=0;
}
void spinLeft()
{
    ma1=0;
    ma2=1;
    mb1=1;
    mb2=0;
}*/
void delay(long int i)
{
    long int j;
    for(j=0;j<i;j++)
    {
        j+=1;
        j-=1;
    }
}
/*
void delayFlash(long int i)
{
	long int j;
	stopNow();
	for(j=0;j<i;j++)
	{
		P2=0xFF;
		delay(7000);
		P2=0x00;
		delay(7000);
	}
}
*/
void beep(long int c)
{
	long int i;
	stopNow();
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
	stopNow();
	for(i=0;i<c;i++)
	{
		buzzer_pin=0;
		delay(3000);
		buzzer_pin=1;
		delay(3000);
	}
} 
/*
void testPWM()
{
	int i=0;
	while(1)
	{
		setMotors(100,100);
		delay(20000);
		for(i=100;i>-100;i--)
		{
			setMotors(i, i);
			delay(2000);
		}
		stopNow();
		delay(20000);
		for(i=-100;i<100;i++)
		{
			setMotors(i, -1*i);
			delay(2000);
		}
		stopNow();
		delay(20000);
	}
} */

void moveGate(int d)
{
	int i;
	for(i=0;i<60;i++)
	{
		servo_pin=1;
		servo_pin=1;
		delay(d);
		//delay(d);
		servo_pin=0;
		delay(210-d);
		//delay(200);
		
	}
}

void testServos()
{
	int j;
	for(j=1;j<400;j+=5)
	{
		moveGate(j);
	}
}

void openGate()
{
	moveGate(7);
}

void closeGate()
{
	moveGate(31);
}

void disp()
{
	stopNow();
	P2=~(((count_sold)<<4)&&(0xF0))||((count_terr)&&(0x0F));
}

long int calib()
{
	int j=0;
	long int sens;
	sens=0;
	s_ON=1;
	
	P3=0xFF;
	P3=0xFF;
	while(cal==1)
	{
		sens++;
		if(sens>10000)break;
	}
	s_ON=0;
	
	delay(10);
	//beep(1);
	//beep(sens/4);
	return sens;
}

void supercalib()
{
	int blackDelay;
	longBeep(1);
	longBeep(1);
	delay(40000);
	longBeep(1);
	whiteDelay=calib();
	if(whiteDelay<=0)whiteDelay=30;

	delay(10000);
	longBeep(2);
	delay(40000);
	longBeep(2);
	blackDelay=calib();
	delay(40000);
	longBeep(3);
	delay(10000);
	if(blackDelay<whiteDelay)
	{
		whiteDelay=1200;
	}
	beep((blackDelay-whiteDelay)/300);
	delay(10000);
	longBeep(4);
	delay(10000);
	beep(blackDelay/300);
	delay(10000);
	longBeep(5);
	delay(10000);
	beep(whiteDelay/300);
	
	while(STOP==1);
}

void readSensor()
{
	s_ON=1;
	delay(10);
	P3=0xFF;
	P3=0xFF;
	smallDelay(whiteDelay);
	P2=P3;
	//delay(10);
	s_ON=0;
	delay(10);
}

void smallDelay(long int d) 
{
	long int i;

	for(i=0;i<d;i++)
	{
		if(i>d) break;
	}
	
}
