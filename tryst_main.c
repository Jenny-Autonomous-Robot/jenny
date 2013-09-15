/*
****************************************************************************************************************

Complete program for Jenny's participation in TRYST 2013 - Maze Busters Robotics Competition at IIT Delhi

----------------------------------------------------------------
Written by : Pranav N. Gour, Arun Subramaniyan, Sujay Narumanchi
---------------------------------------------------------------

This program explores the maze, finds the shortest path, and, in the second iteration of maze traversal, 
reaches the goal following the shortest path.

We are yet to add comments for individual methods. In the meanwhile, we would be
glad to respond to any specific queries via email:

Pranav : f2011075@pilani.bits-pilani.ac.in
Arun : f2011212@pilani.bits-pilani.ac.in
Sujay : f2011025@pilani.bits-pilani.ac.in


******************************************************************************************************************

Hardware Interfacing required:

right motor is A and its PWM is from pin 1^3
left is B, pwm is pin 1^4

P2 is connected to the 8 GPIOs dedicated to the Polulu QTR-8RC sensor array
pins l1-l8 refer to the 8 GPIOs connected to the 8 LEDs that display the status of the sesnor array

NOTE : This program uses an older, more primitive method for reading from the sensor and does not calibrate. 
For a more reliable readSensor method, refer to polulu_qtr8rc_read_calibrate.c in our repository.

*******************************************************************************************************************
*/
	
	
#include <reg51.h>
#include <reg_c51.h>

sbit mb1=P1^5;
sbit mb2=P1^6;
sbit ma1=P1^1;
sbit ma2=P1^7;

sbit s8=P2^0;
sbit s7=P2^1;
sbit s6=P2^2;
sbit s5=P2^3;
sbit s4=P2^4;
sbit s3=P2^5;
sbit s2=P2^6;
sbit s1=P2^7;

sbit l1=P3^7;
sbit l2=P3^6;
sbit l3=P3^5;
sbit l4=P3^4;
sbit l5=P3^3;
sbit l6=P3^2;
sbit l7=P3^1;
sbit l8=P3^0;

sbit sc1 = P0^0;
sbit sc2 = P0^1;
sbit sc3 = P0^2;

sbit d1 = P0^4; //debugging pins
sbit d2 = P0^5;

sbit GO=P0^7;

void backward();
void forward();
void turnLeft();
void spinLeft();
void turnRight();
void spinRight();
void accurateDelay(int);
void stopNow();
void setInitial();
void readSensor();
void delay(long int);
int assignValue();
void delayFlash(int,char);
void assignPID();
void displayPath();
int tag;
tag=0; 
int result;
int lastTurn;
//void setMotors(float, float);
void reduce();	
//int prev_result;
//der, integ;
//float kp,ki,kd;
char path[50],end, MAX_SPEED, NON_PID_SPEED;
char count;
int countBlack;
void go_short();
void main()
{ 
	//char whyturn=1; //1 means right, 2 means left
	
	setInitial();
  //delay(10); 
	while(GO==1);  //so that we know if code resets in the middle
	while(1)
   {
   begin:
		 readSensor();
		 result = assignValue();
		 //delay(1000);
		 //while(kp==0);
		 /*if(result==1111) // dead end case
		 {
			 spinRight(); //TO DO : add a delay here! sometimes, the bot is already satisfying the case below, so the u turn wont happen
			  while(result<40||result>50)
			 {
				 readSensor();
				 result = assignValue();
			 }
			 setMotors(100,100);
	 }
		*/
			 
			 
			if(result==1111)	//all black
			{		
				allblack :
				 forward();
				 delay(1000);
				 stopNow();
				 readSensor();
				 result = assignValue();
				 if(result==1111)
				 {
				 	if(countBlack>20) goto forrest;
					countBlack++;
					goto allblack;
					
				 }

				 else if(result==9999)
				 {
				 	stopNow();
					delay(5000);
					readSensor();
					result = assignValue();
					if(result == 9999)
					{
						forward();
						delay(500);
						readSensor();
						result = assignValue();
						stopNow();
						if(result == 9999)
						{
							countBlack=0;
							goto turnRight;
						}
						else goto allblack;
					}
					else goto allblack;
				  }

				  else if(s1==0 && (s3==1||s4==1||s5==1||s6==1) && s8==0)
				  {
				  	stopNow();
					delay(5000);
					readSensor();
					result = assignValue();
					if(s1==0 && (s3==1||s4==1||s5==1||s6==1) && s8==0)
					{
							countBlack=0;
							goto turnRight;
					}
				  }				  
				  else goto allblack;		
				 /*if(result==9999)
				 {
										 
						 setMotors(100, -100);
						
						while(result<40||result>50)
							{
								 readSensor();
								 result = assignValue();
							}
							path[count++]='R';	
				 }	
				else*/
			/*	if (result == 1111)
				{
				setMotors(100,100);
				 delay(10000);
				 stopNow();
				 readSensor();
				 result = assignValue();
				 if(result==1111)
				 {
										 
						 //stopNow();
						 delayFlash(1, 0x00);
						end=1;	
					 goto forrest; 
				 }
			 	 }	*/			 
				
	turnRight:	turnRight(); //turn right
				 while(1)
				  {										
					 readSensor();
					 //result = assignValue();
					 if(s4==0 && s5==0) //CHANGED
					 {
					 	stopNow();
						delay(2000);
						readSensor();
						if(s4==0 && s5==0)break;
						else turnRight();
					 }

				  } 
				 spinRight();
				 while(1)
				 {
				 	readSensor();
					if(s1==0 && ((s4==1)||(s5==1)) && s8==0) break;  //this line used to be if(s1==0 && ((s4==1)||(s5==1)) && s8==0) break;  
				 }				
					forward(); //now frwrd
					path[count++]='R';	
			}
		
					
			else if(result==9911)
			{
				d1=0;
				d2=0;
					 
			  foundRightTurn:
				 forward();
				 delay(1000);
				 stopNow();
			 
				 readSensor();
				 result = assignValue();
				 if(result==1111) goto allblack;
				 else if((s1==0)&&((s3==1)||(s4==1)||(s5==1)||(s6==1))&&(s8==0))
				 {
				 	 stopNow();
						delay(500);
						readSensor();
						result = assignValue();
						if(s1==0 && (s3==1||s4==1||s5==1||s6==1) && s8==0)
						{
								forward();
								delay(500);
								stopNow();
								readSensor();
								result = assignValue();
								if(s1==0 && (s3==1||s4==1||s5==1||s6==1) && s8==0)
								{
										path[count++]='R';
										goto handleRightTurn;
								}
								else goto foundRightTurn;
							}
							else goto foundRightTurn; 					  
				 }
				 else if(result==9911)
				 {
				 	d1=1;
					d2=0;
				 	goto foundRightTurn;
				 } 
				 
				 else if(result==9999) 
				 {
					stopNow();
					delay(500);
					readSensor();
					result = assignValue();
					if(result == 9999)
					{
						forward();
						delay(500);
						readSensor();
						result = assignValue();
						stopNow();
						if(result == 9999)
						{
							goto handleRightTurn;
						}
						else goto foundRightTurn;
					}
					else goto foundRightTurn;
				}
				 else
				 {
				 	d1=1;
					d2=1;
					goto foundRightTurn;
				 }
				 
				 
				 
				 /*while(s8==1) setMotors(100,100);
				 stopNow();
  			 	 readSensor();
				 result = assignValue();
				 if(result!=9999)
				 {
						//store : we took a right
				 }
				*/						 
				 
handleRightTurn:
				 turnRight();
				 
				 while(1)
				  {										
					 readSensor();
					 //result = assignValue();
					 if(s4==0 && s5==0)
					 {
					 	stopNow();
						delay(2000);
						//d1=1;
					//	d2=0;
						readSensor();
						if(s4==0 && s5==0)break;
						else turnRight();
					 }

				  } 
				 spinRight();
				 while(1)
				 {
				 	readSensor();
					//d1=1;
					//d2=1;
					if(s1==0 && ((s4==1)||(s5==1)) && s8==0) break; 
				 }
					//d1=0;
				//	d2=0;
					forward(); //now frwrd
			}

			else if ( result == 1199)
			{
d1=1;
d2=1;
				
foundLeftTurn:
				 forward();
				 delay(1000);
				 stopNow();
			 
				 readSensor();
				 result = assignValue();
				 if(result==1111) goto allblack;
				 else if((s1==0)&&((s3==1)||(s4==1)||(s5==1)||(s6==1))&&(s8==0))
				 {
				  stopNow();
					delay(5000);
					readSensor();
					result = assignValue();
					if(s1==0 && (s3==1||s4==1||s5==1||s6==1) && s8==0)
					{
							forward();
							delay(3000);
							stopNow();
							readSensor();
							result = assignValue();
							if(s1==0 && (s3==1||s4==1||s5==1||s6==1) && s8==0)
							{
									path[count++]='F';
									forward();
									goto begin;
							}
							else goto foundLeftTurn;
						}
					  else goto foundLeftTurn;
				 }
				 else if(result==1199)
				 {
				 	d1=1;
					d2=1;
				 	goto foundLeftTurn;
				 } 
				 
				 else if(result==9999)
				 {
					stopNow();
					delay(3000);
					readSensor();
					result = assignValue();
					if(result == 9999)
					{
						forward();
						delay(500);
						readSensor();
						result = assignValue();
						stopNow();
						if(result == 9999)
						{
							d1=1;
							d2=0;
							goto handleLeftTurn;
						}
						else goto foundLeftTurn;
					}
					else goto foundLeftTurn;
				}
				 else
				 {
				 	//d1=1;
					//d2=1;
					goto foundLeftTurn;
				 }
				 
				 
				 
				 /*while(s8==1) setMotors(100,100);
				 stopNow();
  			 	 readSensor();
				 result = assignValue();
				 if(result!=9999)
				 {
						//store : we took a right
				 }
				*/						 
				 
				 handleLeftTurn:
				 turnLeft();
				 
				 while(1)
				  {										
					 readSensor();
					 //result = assignValue();
					 if(s4==0 && s5==0)
					 {
					 	stopNow();
						delay(2000);
						readSensor();
						//d1=1;
					//	d2=0;
						if(s4==0 && s5==0)break;
						else turnLeft();
					 }

				  } 
				 spinLeft();
				 while(1)
				 {
				 	readSensor();
					//d1=1;
					//d2=1;
					if(s1==0 && ((s4==1)||(s5==1)) && s8==0) break; 
				 }
					//d1=0;
				//	d2=0;
					forward(); //now frwrd
			}
									
			else if(result==9999)
			{
				//d1=1;
				//d2=1;
				//backward();
				//delay(10000);
				if(lastTurn<30) spinLeft();
				else if(lastTurn>60)
				{
					spinRight();
					d1=1;
					d2=0;
				}
				else 
				{
					d1=0;
					d2=1;
					stopNow();
					delay(5000);
					readSensor();
					result = assignValue();
					if(result == 9999)
					{
						//d1=0;
						//d2=0;
						forward();
						delay(5000);
						readSensor();
						result = assignValue();
						stopNow();
						if(result == 9999)
						{	
							d1=0;
							d2=0;
							forward();
							delay(6000);
							//delay(20000);
							spinRight();
							while((result<40)||(result>50))
							{
								 readSensor();
								 result = assignValue();
							}
						 forward();
						path[count++]='U';	//store u turn 
						}
				 	}
				}
			}

		 	else if((result>=40)&&(result<=50))
			{
				forward();
			}
			else if(result<40)
			{
				d1=0;
				d2=0;
				lastTurn=result;
				spinLeft();
			}		
			else if(result>50)
			{
				lastTurn=result;
				spinRight();
			}
			
		}
forrest:
{
	//char i;
//	i=0;
	displayPath();
	while(GO==0);
	reduce();
	delayFlash(1, 0x00);
	displayPath();	
	go_short();
	while(1);

}
}

void displayPath()
{
	int k;
	P3=count;
	for(k=0;k<count;k++)
	{
			if(path[k]=='F')
				delayFlash(1,0xC3);
			if(path[k]=='R')
				delayFlash(1,0xF0);
			if(path[k]=='L')
				delayFlash(1,0x0F);
			if(path[k]=='U')
				delayFlash(1,0xAA);
			if(path[k]==0x00) break;
	}
}
/*void setMotors(float leftval, float rightval)
{
	if(leftval>=0)
	{
		leftval=(((MAX_SPEED-255.0)*leftval)/100.0) + 255.0;
		mb1=0;
		mb2=1;
	}
	else
	{
		leftval=(((MAX_SPEED-255.0)*((-1)*leftval))/100.0) + 255.0;
		mb1=1;
		mb2=0;
	}
	if(rightval>=0)
	{
		rightval=(((MAX_SPEED-255.0)*rightval)/100.0) + 255.0;
		ma1=0;
		ma2=1;
	}
	else
	{
		rightval=(((MAX_SPEED-255.0)*((-1)*rightval))/100.0) + 255.0;		
		ma1=1;
		ma2=0;
	}
	CCAP0L=(char)rightval;
	CCAP0H=(char)rightval;
	CCAP1L=(char)leftval;
	CCAP1H=(char)leftval;
}
*/
/*void setOldPWM()
{
	CCAP0L=(char)NON_PID_SPEED;
  CCAP0H=(char)NON_PID_SPEED;
	CCAP1L=(char)NON_PID_SPEED;
  CCAP1H=(char)NON_PID_SPEED;
}*/

int assignValue()
{
	int den=0;
	int res=0;
		
	if((s1==1)&&((s4==1)||(s5==1))&&(s8==1)) //bring back s1 here
	{
		return 1111;//to check if all the sensors are black and if true move forward and check
	}
	else if((s1==0)&&(s2==0)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==0)&&(s8==0)) //bring back s1 here
	{
		return 9999;//to check if it is completely white and hence take the turn previously taken
	}

	else if((s1==1)&&(((s2==1)&&(s3==1))||((s3==1)&&(s4==1))||((s2==1)&&(s4==1)))&&(s7==0)&&(s8==0))	 //CHANGED CHANGED CHANGED
	{
		return 1199;
	}
	
	else if((s1==0)&&(s2==0)&&(((s5==1)&&(s6==1))||((s6==1)&&(s7==1))||((s5==1)&&(s7==1)))&&(s8==1))	 //CHANGED CHANGED CHANGED
	{
		return 9911;
	}
	
	else // IMP : this is where to implement PID
	{		
			den=(int)s2+(int)s3+(int)s4+(int)s5+(int)s6+(int)s7;
	
			res=((((int)s2*20)+((int)s3*30)+((int)s4*40)+((int)s5*50)+((int)s6*60)+((int)s7*70))/den);
	
			return res;
	}
}
void readSensor()
{
    delay(20); //this delay is NECESSARY. But why? Perhaps because it gives extra time for charging. /CHANGED CHANGED CHANGED
    
    TL1=0xF5;
    TH1=0xFF; //10 microsec delay

    P2=0xFF; //output mode to charge capacitor
    P2=0xFF; //make sure capacitor is charged
    TR1=1;  // start timer
    while(TF1==0);//delay
    TR1=0; // end timer
    TF1=0; // VERY IMPORTANT!!!!
    
    delay(20); //This delay, if used instead of the previous one, still makes the system work. Not sure why. /CHANGED CHANGED CHANGED
    
    P2=0xFF; //now in input mode
    
    TL1=0x6D;
    TH1=0xDB;// for 10 ms wait
    TR1=1; // start timer
    while(TF1==0); // checking for overflow
    TR1=0; // end timer
    TF1=0; // VERY IMPORTANT!!!!!
    
		P3=P2;
	
		//to show output
    delay(20);			 //CHANGED CHANGED CHANGED
//value is now in P2
}

void setInitial()
{
	MAX_SPEED=200;
	count = 0;
	end=0;
	//kp=4.0;
	//ki=0.0;
	//kd=0.0;
	//integ=0;
	//der=0;
	//prev_result=0;
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
    CCAP0L=0xC8;
    CCAP0H=0xC8;
    CCAPM1=0x42;
    CCAP1L=0xC8;
    CCAP1H=0xC8;
    CCON=0x40;
/*
	if(sc1==0 && sc2==0 &&sc3==0)MAX_SPEED = 200;
	else if(sc1==0 && sc2==0 &&sc3==1)MAX_SPEED = 175 ;
	else if(sc1==0 && sc2==1 &&sc3==0)MAX_SPEED = 150;
	else if(sc1==0 && sc2==1 &&sc3==1)MAX_SPEED = 125;
	else if(sc1==1 && sc2==0 &&sc3==0)MAX_SPEED = 100;
	else if(sc1==1 && sc2==0 &&sc3==1)MAX_SPEED = 75;
	else if(sc1==1 && sc2==1 &&sc3==0)MAX_SPEED = 50;
	else if(sc1==1 && sc2==1 &&sc3==1)MAX_SPEED = 25;*/
	NON_PID_SPEED = MAX_SPEED;
	
	countBlack=0;
	d1=1;
	d2=1;		
}
    

void stopNow()
{
//		setOldPWM();
    ma1=1;
    ma2=1;
    mb1=1;
    mb2=1;
}
    
void forward()
{
    ma1=0;
    ma2=1;
    mb1=0;
    mb2=1;
}
void backward()
{
	//	setOldPWM();
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
void delayFlash(int i, char a)
{
	int j;
	stopNow();
	for(j=0;j<i;j++)
	{
		P3=0xFF;
		delay(15000);
		P3=a;
		delay(15000);
	}
}

void reduce()
{	 
	int i,j,count2;	
	 
	 count2=0;	
	for(i=0;i<count;i++)
	 {
		 if(path[i]=='U') count2++;
	 }
	
	while(count2!=0)
	 {
		for(i=0;i<count;i++)
		{
			if(path[i]=='U')
			{
					if((path[i-1]=='R'&&path[i+1]=='L')||(path[i-1]=='L'&&path[i+1]=='R')||(path[i-1]=='F'&&path[i+1]=='F'))
					{
							path[i-1]='U';
							for(j=i;j<(count-2);j++)
							{	
								path[j]=path[j+2];
							}
							path[j]=0;
							i--;
							count-=2;
						  
					}
					
					else if((path[i-1]=='R'&&path[i+1]=='R')||(path[i-1]=='L'&&path[i+1]=='L'))
					{
							path[i-1]='F';
							for(j=i;j<(count-2);j++)
							{	
								path[j]=path[j+2];
							}
							path[j]=0;
							i--;
							count-=2;
						  count2--;
					}
					
					else if((path[i-1]=='R'&&path[i+1]=='F')||(path[i-1]=='F'&&path[i+1]=='R'))
					{
							path[i-1]='L';
							for(j=i;j<(count-2);j++)
							{	
								path[j]=path[j+2];
							}
							path[j]=0;
							i--;
							count-=2;
						  count2--;
					}
					
					else if((path[i-1]=='L'&&path[i+1]=='F')||(path[i-1]=='F'&&path[i+1]=='L'))
					{
							path[i-1]='R';
							for(j=i;j<(count-2);j++)
							{	
								path[j]=path[j+2];
							}
							path[j]=0;
							i--;
							count-=2;
						  count2--;
					}
				}
			}
		}	
	}

	void go_short()
	{
		char i=0;
	//	setInitial();
  //delay(10); 
	delayFlash(count, 0xCC);
	while(i<count)
   {

		 readSensor();
		 result = assignValue();
		 if((result==1111)||(result==9911)||(result==1199))
		 {
		 	if(path[i]=='R') 
			{
				turnRight();
				 while(1)
				  {										
					 readSensor();
					 //result = assignValue();
					 if(s4==0 && s5==0)
					 {
					 	stopNow();
						delay(2000);
						readSensor();
						if(s4==0 && s5==0)break;
						else turnRight();
					 }

				  } 
				 spinRight();
				 while(1)
				 {
				 	readSensor();
					if(s1==0 && ((s4==1)|(s5==1)) && s8==0) break; 
				 }				
					forward(); //now frwrd
			}
			
				else if(path[i]=='L') 
			{
				stopNow();
				delayFlash(1,0x33);
				turnLeft(); 
				 while(1)
				  {										
					 readSensor();
					 //result = assignValue();
					 if(s4==0 && s5==0)
					 {
					 	stopNow();
						delay(2000);
						readSensor();
						if(s4==0 && s5==0)break;
						else turnLeft();
					 }

				  } 
				 spinLeft();
				 while(1)
				 {
				 	readSensor();
					if(s1==0 && ((s4==1)||(s5==1)) && s8==0) break; 
				 }				
					forward(); //now frwrd
			}	
		   else if(path[i]=='F')
		   { 
		   		forward();
				while(1)
				{
					if(s1==0 && (s3==1||s4==1||s5==1||s6==1)&&s8==0)
					{
						break;
					}
				}
	       		
		   }
		   i++;	
		
			
		}

		else if((result>=40)&&(result<=50))
		{
			forward();
		}
		else if(result<40)
		{
			lastTurn=result;
			spinLeft();
		}		
		else if(result>50)
		{
			lastTurn=result;
			spinRight();
		}
		
	}

	while(1)
	{
		readSensor();
		result=assignValue();
		if(result==1111)
		{	
			 stopNow();
			 delay(1000);
			 forward();
			 delay(1000);
			 stopNow();
			 readSensor();
			 result=assignValue();
			 if((result==1111))
			 {
			  break;
			 }	
			 
		 }
	
		else if((result>=40)&&(result<=50))
		{
			forward();
		}
		else if(result<40)
		{
			lastTurn=result;
			spinLeft();
		}		
		else if(result>50)
		{
			lastTurn=result;
			spinRight();
		}
		
	}
	stopNow();  
}	
