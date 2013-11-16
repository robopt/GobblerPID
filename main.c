/////////////////////
// GobblerPID main //
// Edward Mead     //
/////////////////////
#include <hidef.h>        /* common defines and macros */
#include <mc9s12dg256.h>     /* derivative information */
#include "main.h"
#pragma LINK_INFO DERIVATIVE "mc9s12dg256b"
#include "main_asm.h" /* interface to the assembly module */

////////////////////////////////////////
// Initalize                          //
// Ports, Interrupts, Motor and Servo //
////////////////////////////////////////
void portInit(void)
{
    PLL_init(); // set system clock frequency to 24 MHz
    
    //Direction registers
    DDRH = 0x00;  // Port H is input
    DDRB = 0x0F; // Port B starts as output
    PORTB = 0x00; //turn all LED's OFF
    DDRT = 0xFF;  // port T7 is input rest is output
    
    //motors
    PTT_PTT0 = 0; //default
    PTT_PTT1 = 0; //default
    PTT_PTT2 = 0; //default
    PTT_PTT3 = 0; //default
    motor0_init(); //left motor
    motor1_init(); //right motor
    motor7_init(); //pickup motor
    
    //servo
    servo54_init(); //initialize servo
    set_servo54(2600); //servo default location
    
    BASE = BASEOUTER;
    gWallCurrent = gWallOuter;
    wallTurnCurrent = wallTurnOuter;
    wallBrakeCurrent = wallBrakeOuter;
    
    //initialize a/d
    ad0_enable(); //enable a/d converter w/ interrupt
    RTI_init(); //turn on real time interrupt
    
}


void main(void)
{
    portInit();  //initialize ports, interrupts, and motors
    for(;;); //do nothing forever (interrupt driven)
    
}



///////////
// Setup //
///////////
void setup()
{
    if (PORTB_BIT7)
    {
        int i;
        PORTB_BIT0 = PORTB_BIT1 = PORTB_BIT2 = PORTB_BIT3 = 0; //LEDS OFF
        for(i = 0; i < 10; i++)
        {
            //flash LEDs for 250ms to show start
            PORTB_BIT0 = ~PORTB_BIT0;
            PORTB_BIT1 = ~PORTB_BIT1;
            PORTB_BIT2 = ~PORTB_BIT2;
            PORTB_BIT3 = ~PORTB_BIT3;
            ms_delay(25);
            
        }
        
        motor7(0xFF); //Full power to pickup motor
        PORTB_BIT0 = PORTB_BIT1 = PORTB_BIT2 = PORTB_BIT3 = 0; //LEDS OFF
        isSetup = 0x01; //we are ready to roll!
        PTT_PTT1 = 1; //Spin Motor 2
        PTT_PTT2 = 1; //Motor EN
        
    }
    else
    {
        averageSensors();
        //deltaSpeed = s4avg - lastFront;
        //lastFront = s4avg;
        if (s4avg > wallTurnCurrent)
        {
            PORTB_BIT0 = 1; //enable 1st LED to show front sensor within outer turn value
            
        }
        else
        {
            PORTB_BIT0 = 0; //enable 1st LED to show front sensor outside outer turn value
            
        }
        
        if (s5avg > gWallCurrent)
        {
            PORTB_BIT1 = 1; //disable 1st LED to show front sensor outside outer turn value
            
        }
        else
        {
            PORTB_BIT1 = 0; //disable 1st LED to show front sensor outside outer turn value
            
        }
        
        if (s6avg > lineValueThresh)
        {
            PORTB_BIT2 = 1; //enable 3rd LED to show line sensor within outer line value
            
        }
        else
        {
            PORTB_BIT2 = 0; //disable 3rd LED to show line sensor outside outer line value
            
        }
        
        if (s7avg < 100)
        {
            PORTB_BIT3 = 1; //enable 3rd LED to show line sensor within outer line value
            
        }
        else
        {
            PORTB_BIT3 = 0; //disable 3rd LED to show line sensor outside outer line value
            
        }
        
        
    }
    
    
}


//////////////////////////
// Forward Proportional //
// Wall follow                //
//////////////////////////
void forwardP()
{
    error = gWallCurrent - s5avg; //instant error e
    routput = (int)((BASE-(s4avg>>2))-(Kp*error));
    loutput = (int)((BASE-(s4avg>>2))+(Kp*error));
    
    // saturation logic
    if(routput > MAX)  // no motor values > 255 or < 0
    routput = MAX;
    else if(routput <= MIN)
    routput = MIN;
    if(loutput > MAX)  // no motor values > 255 or < 0
    loutput = MAX;
    else if(loutput <= MIN)
    loutput = MIN;
    
    
    if (loutput == MIN)
    {
        PTT_PTT3 = 1; //Motor L1
        PTT_PTT4 = 1; //Motor L2
        
    }
    else
    {
        PTT_PTT3 = 0; //Motor L1
        PTT_PTT4 = 1; //Motor L2
        
    }
    
    if (routput == MIN)
    {
        PTT_PTT6 = 1; //Motor R1
        PTT_PTT7 = 1; //Motor R2
        
    }
    else
    {
        PTT_PTT6 = 0; //Motor R1
        PTT_PTT7 = 1; //Motor R2
        
    }
    
    //pwm
    motor0(routput); //Right motor
    motor1(loutput); //Left motor
    
}




//////////////////////////
// Forward Proportional //
// Line follow                //
//////////////////////////
char lostLine = 0; //did we lose the line?
void forwardLine()
{
    s1 = ad0conv(2); //line sensor
    s2 = ad0conv(3); //line sensor
    s3 = ad0conv(6); //line sensor
    s4 = ad0conv(4); //line sensor
    senSum = (s1+s2+s3); //sum
    sum = ((s1*10) + (s2*40) + (s3*70))/senSum; //weighted sum
    
    if (senSum < 50)
    {
        lostLine = 1;
        if (lastSide == 1)
        {
            sum = 59; //max right
            PTT_PTT3 = 1; //Motor L1
            PTT_PTT4 = 0; //Motor L2
            PTT_PTT6 = 0; //Motor L1
            PTT_PTT7 = 1; //Motor L2
            motor0(LINEBASE); //right motor
            motor1(LINEBASE>>1); //left motor
            
        }
        
        else
        {
            sum = 13; //max left
            PTT_PTT3 = 0; //Motor L1
            PTT_PTT4 = 1; //Motor L2
            PTT_PTT6 = 1; //Motor L1
            PTT_PTT7 = 0; //Motor L2
            motor0(LINEBASE>>1); //right motor
            motor1(LINEBASE); //left motor
            
        }
        
        
    }
    else
    {
        error = lineOptimum - sum; //instant error
        if (error < 0)  //record last side of line are on
        lastSide = 0; //left side
        else if (error > 0)  //record last side of line are on
        lastSide = 1; //right side
        deltaError = error-lastError; //rate of change of error
        lastError = error; //save last value
        routput = (int)((LINEBASE)+(KpLine*error)+(KdLine*deltaError));// + ((KiLine)*sumError));
        loutput = (int)((LINEBASE)-(KpLine*error)+(KdLine*deltaError));// + ((KiLine)*sumError));
        
        //saturation logic
        if(routput > LINEMAX)
        routput = LINEMAX;
        else if(routput < MIN)
        routput = MIN;
        if(loutput > LINEMAX)
        loutput = LINEMAX;
        else if(loutput < MIN)
        loutput = MIN;
        
        if (loutput == MIN)
        {
            PTT_PTT3 = 1; //Motor L1
            PTT_PTT4 = 0; //Motor L2
            loutput = LINEBASE;
            routput = LINEMAX;
            
        }
        else
        {
            PTT_PTT3 = 0; //Motor L1
            PTT_PTT4 = 1; //Motor L2
            
        }
        
        if (routput == MIN)
        {
            PTT_PTT6 = 1; //Motor L1
            PTT_PTT7 = 0; //Motor L2
            routput = LINEBASE;
            loutput = LINEMAX;
            
        }
        else
        {
            PTT_PTT6 = 0; //Motor L1
            PTT_PTT7 = 1; //Motor L2
            
        }
        
        
        //pwm
        motor0(routput); //right motor
        motor1(loutput); //left motor
        
    }
    
    
}



//WORKING
///////////////////////////
// Average sensor values //
///////////////////////////
void averageSensors()
{
    int i; //loop variable
    s4total = s5total = s6total = s7total=0; //reset totals to 0
    if (sensorAvgIndex > 50)
    {
        //limit to last 50 readings (reduce loop time)
        for (i = sensorAvgIndex-50; i < (sensorAvgIndex); i++) //total 25 of most recent values for each sensor
        {
            s4total += s4a[i]; //add front to total
            s5total += s5a[i]; //add side (long) to total
            s6total += s6a[i]; //add line to total
            s7total += s7a[i]; //add side (short) to total
            
        }
        i = 50;
        
    }
    else //we have less than 50 values, total all of them
    {
        for (i = 0; i < (sensorAvgIndex); i++)  //total all values
        
        {
            s4total += s4a[i]; //add front to total
            s5total += s5a[i]; //add side (long) to total
            s6total += s6a[i]; //add line to total
            s7total += s7a[i]; //add side (short) to total
            
        }
        
    }
	
    s4avg = s4total/i; //update average
    s5avg = s5total/i; //update average
    s6avg = s6total/i; //update average
    s7avg = s7total/i; //update average
    sensorAvgIndex = 0; //reset sensor storage index to 0
}



////////////////////////////////////////
// Real time interrupt                //
// Higher priority than ATD interrupt //
// Modified to 2 ms                   //
////////////////////////////////////////
void interrupt 7 RTIhandler()
{
    if (isSetup == 0x00)
    {
        setup();
    }
    else
    {
        averageSensors();
		
        if (lineCount == 9)
        {
            dumpApproach();
            lastLine++;
            clear_RTI_flag();
            return;
            
        }
        else if (lastLine >= lastThresh && s6avg > lineValueThresh)
        {
            brakeEnabled = 1;
            lastLine = 0; // reset last line threshhold
            PORTB = ++lineCount; //display lines on LED's
            if (lineCount < 4)
            {
                if (lineCount == 2)
                {
                    wallBrakeOuter +=4;  //compensation for momentum/acceleration
                    wallTurnCurrent += 2;   //compensation for momentum/acceleration
                    
                }
                
                if (lineCount == 3)
                {
                    wallBrakeOuter +=1;  //compensation for momentum/acceleration
                    wallTurnCurrent += 1;  //compensation for momentum/acceleration
                    
                    
                }
                
                
            }
            else if (lineCount == 4)
            {
                Kp = KpInner; //increase Kp to be more responsive on inner turns
                BASE = BASEINNER;
                gWallCurrent = gWallInner; //change to inner wall follow value
                wallTurnCurrent = wallTurnInner; //change to inner turn value
                wallBrakeCurrent = wallBrakeInner+3;  //compensation for momentum/acceleration
                brakeThresh = brakeThreshInner;
                BASETURN = BASETURNINNER;
                
            }
            else if (lineCount == 5)
            {
                gWallInner -= 1;
                wallBrakeCurrent = wallBrakeInner;
                wallTurnCurrent = wallTurnCurrent-3;  //compensation for momentum/acceleration
                
            }
            else if (lineCount == 6)
            {
                BASE +=10;
                wallTurnCurrent = wallTurnCurrent+4;  //compensation for momentum/acceleration
                wallBrakeCurrent += 3;  //compensation for momentum/acceleration
                
            }
            else if (lineCount == 7)
            {
                BASE +=10;
                wallTurnCurrent = wallTurnCurrent; // other side of angle of inner loop
                
            }
            else if (lineCount == 8)
            {
                BASE = BASEINNER;
                wallTurnCurrent = wallTurnCurrent + wallDumpApproach; // get further from box to collect
                gWallCurrent = gWallCurrent + wallDumpApproach;
                wallBrakeCurrent = wallBrakeOuter;
                
            }
            else if (lineCount == 9)
            {
                PTT_PTT1 = 0;
                ms_delay(200); //wait to go over line completely
                ad0_enableLine();
                while(ad0conv(3) < lineValueThresh) //back up  (right faster) until we hit line again
                backSplit(BASELBKUP,BASERBKUP);
                lastLine++;
                clear_RTI_flag();
                return;
                
            }
            
            
        }
        
        
        
        
        if (lineCount < 4 && s4avg > wallTurnCurrent)
        {
            brakeEnabled = 0x00;
            leftManual(BASETURN,BASETURN);
            
        }
        else if (lineCount >= 4 && s4avg > wallTurnCurrent)
        {
            //latch inner turn
            turnEnabled = 1;
            leftManual(BASETURN,BASETURN);
            
        }
        else if (turnEnabled == 1 && s4avg > wallTurnInnerExit)
        {
            //continue latch until we reach our value
            leftManual(BASETURN,BASETURN);
            
        }
        else
        {
            turnEnabled = 0;
			//check if we need to start anti-lock braking
            if (brakeEnabled == 0x01 && s4avg > wallBrakeCurrent && ++brakeCount > brakeThresh)
            {
                back(BASEBRAKE); //Reverse current in motors
                brakeCount = 0;
            }
            else
            {
                forwardP();
            }
            
            
        }
        
        lastLine++;
        
    }
    
    clear_RTI_flag();
}



//WORKING
////////////////////////////////////////////////
// A/D Converter interrupt                                    //
// enabled through custom assembly subroutine //
////////////////////////////////////////////////
void interrupt 22 ANhandler()
{
    s4 = ATD0DR0L; // ATD converter 0 data register 0 lower 8 bytes
    s5 = ATD0DR1L; // ATD converter 0 data register 1 lower 8 bytes
    s6 = ATD0DR2L; // ATD converter 0 data register 2 lower 8 bytes
    s7 = ATD0DR3L; // ATD converter 0 data register 3 lower 8 bytes
    s4a[sensorAvgIndex] = s4; // store front IR value in sensor array
    s5a[sensorAvgIndex] = s5; // store side IR (LONG) value in sensor array
    s6a[sensorAvgIndex] = s6; // store line IR in sensor array
    s7a[sensorAvgIndex++] = s7; // store side IR (SHORT) in sensor array
}




//////////////////////////////////////////////
// Dump the balls into the box and back up. //
//////////////////////////////////////////////
void dumpApproach()
{
    int i;
    PORTB_BIT2 = ~PORTB_BIT2; //flash 3rd LED to show dump sequence
    forwardLine(); //follow line
    s7 = ad0conv(7); //read front sensor manually
    if (s7 < 100)
    {
        //digital sensor triggered
        set_servo54(1500); //dump
        PTT_PTT1 = 1;
        for(i = 0; i < 140; i++)
        {
            forwardLine(); //follow line
            ms_delay(10);
            
        }
        
        back(0x8F); //back up @ full speed
        ms_delay(300); //wait to move away from box
        stop(); //stop motors
        PTT_PTT1 = 0; //disable pickup wheel
        PTT_PTT2 = 0; //disable motor controller
        for(;;)
        {
            PORTB_BIT0 = ~PORTB_BIT0; //flash LEDS to show completed
            PORTB_BIT1 = ~PORTB_BIT1; //flash LEDS to show completed
            PORTB_BIT2 = ~PORTB_BIT2; //flash LEDS to show completed
            PORTB_BIT3 = ~PORTB_BIT3; //flash LEDS to show completed
            ms_delay(10); //delay so LED's shut off
            forward(0x00); //drop duty cycle to 0 to be safe
            
        }
        
        
    }
    
    
}




////////////////////////
// Movement Functions //
////////////////////////
void stop()
{
    motor0(0xFF);
    motor1(0xFF);
    PTT_PTT3 = 1; //Motor L1
    PTT_PTT4 = 1; //Motor L2
    PTT_PTT6 = 1; //Motor R1
    PTT_PTT7 = 1; //Motor R2
    
}


void forward(char speed)
{
    motor0(speed);
    motor1(speed);
    PTT_PTT3 = 0; //Motor L1
    PTT_PTT4 = 1; //Motor L2
    PTT_PTT6 = 0; //Motor R1
    PTT_PTT7 = 1; //Motor R2
    
}


void left(char rspeed)
{
    motor0(rspeed);
    motor1(0xFF);
    PTT_PTT3 = 1; //Motor L1
    PTT_PTT4 = 1; //Motor L2
    PTT_PTT6 = 0; //Motor R1
    PTT_PTT7 = 1; //Motor R2
    
}


void leftManual(char rspeed, char lspeed)
{
    motor0(rspeed);
    motor1(lspeed);
    PTT_PTT3 = 1; //Motor L1
    PTT_PTT4 = 0; //Motor L2
    PTT_PTT6 = 0; //Motor R1
    PTT_PTT7 = 1; //Motor R2
    
}


void back(char speed)
{
    motor0(speed);
    motor1(speed);
    PTT_PTT3 = 1; //Motor L1
    PTT_PTT4 = 0; //Motor L2
    PTT_PTT6 = 1; //Motor R1
    PTT_PTT7 = 0; //Motor R2
    
}


void backSplit(char rspeed, char lspeed)
{
    motor0(rspeed);
    motor1(lspeed);
    PTT_PTT3 = 1; //Motor L1
    PTT_PTT4 = 0; //Motor L2
    PTT_PTT6 = 1; //Motor R1
    PTT_PTT7 = 0; //Motor R2
    
}


void backLeft(char rspeed, char lspeed)
{
    motor0(rspeed);
    motor1(lspeed);
    PTT_PTT3 = 1; //Motor L1
    PTT_PTT4 = 0; //Motor L2
    PTT_PTT6 = 0; //Motor R1
    PTT_PTT7 = 0; //Motor R2
    
}