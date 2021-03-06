#pragma config(Motor,  port2,           right_wheels,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           left_wheel_front, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           upper_right_arm_motor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           upper_left_arm_motor, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           lower_right_arm_motor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           lower_left_arm_motor, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           claw_up_down,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           claw_rotate,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          left_wheel_back, tmotorNone, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

#define MAXSPEED 127
#define CLAWROTATESPEED 45

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	/*
 	motor[right_wheels] = 60;
 	motor[left_wheel_front] = 60;
    motor[left_wheel_back] = 127;
    wait1Msec(2200);
    motor[right_wheels] = 0;
    motor[left_wheel_front] = 0;
    motor[left_wheel_back] = 0;
    */
    int placehold = 0;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {

		// ### DEFINE VARIABLES AND SPEEDS ### //
		int right_wheel_speed = -vexRT[Ch2] + 67;
		int left_wheel_front_speed = -vexRT[Ch3] + 67;
		int left_wheel_back_speed = vexRT[Ch3];
		int right_arm_speed = MAXSPEED;
		int left_arm_speed = MAXSPEED;
		int claw_up_down_speed = MAXSPEED;
		int claw_rotate_speed = CLAWROTATESPEED;
		
		if(vexRT[Ch2] >= -15 && vexRT[Ch2] <= 15){
			right_wheel_speed = 0;
		}
		if(vexRT[Ch3] >= -15 && vexRT[Ch3] <= 15){
			left_wheel_front_speed = 0;
		}
		
		if(right_wheel_speed > 127){
			right_wheel_speed = 127;
		}else if(right_wheel_speed < -127){
			right_wheel_speed = -127;
		}
		
		if(left_wheel_front_speed > 127){
			left_wheel_front_speed = 127;
		}else if(left_wheel_front_speed < -127){
			left_wheel_front_speed = -127;
		}


		// ### MOVE BASE MOTORS ### //
		motor[right_wheels] = right_wheel_speed;
		motor[left_wheel_front] = left_wheel_front_speed;
		motor[left_wheel_back] = left_wheel_back_speed;

		
		// ### MOVE 4BAR MOTORS ### //
		if(vexRT[Btn6U] == 1){
			motor[upper_right_arm_motor] = right_arm_speed;
			motor[lower_right_arm_motor] = right_arm_speed;
			motor[upper_left_arm_motor] = left_arm_speed;
			motor[lower_left_arm_motor] = -left_arm_speed;
		}else if(vexRT[Btn6D] == 1){
			motor[upper_right_arm_motor] = 0;
			motor[lower_right_arm_motor] = 0;
			motor[upper_left_arm_motor] = 0;
			motor[lower_left_arm_motor] = 0;
		}else{
			motor[upper_right_arm_motor] = 15;
			motor[lower_right_arm_motor] = 15;
			motor[upper_left_arm_motor] = 15;
			motor[lower_left_arm_motor] = -15;
		}
	//	untilPotientometerGreaterThen(

		//### CLAW CONTROLS ###//
		if(vexRT[Btn8L] == 1){
			motor[claw_up_down] = claw_up_down_speed;
		}else if(vexRT[Btn8R] == 1){
			motor[claw_up_down] = -claw_up_down_speed;
		}else{
			motor[claw_up_down] = 0;
		}

		if(vexRT[Btn5U] == 1){
			motor[claw_rotate] = claw_rotate_speed;
		}else if(vexRT[Btn5D] == 1){
			motor[claw_rotate] = -claw_rotate_speed;
		}else{
			motor[claw_rotate] = 0;
		}
		
	/*	if(vexRT[Btn7L] == 1){
			motor[right_wheels] = 60;
 			motor[left_wheel_front] = 60;
    		motor[left_wheel_back] = 127;
    		wait1Msec(2200);
   		 	motor[right_wheels] = 0;
   		 	motor[left_wheel_front] = 0;
    		motor[left_wheel_back] = 0;
	}
	*/
  }
}


