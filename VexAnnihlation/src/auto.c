/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
void pidcontrol(int target) {
	//reason for the "2" is for right side
	//final power vals for motors
	float speed = 0;
	float speed2 = 0;
	//how far the robot is from the target and stores previous value
	float error = 0;
	float error2= 0;
	float preverror= 0;
	float preverror2= 0;
	//actual pid values, Kp for proportional constant value and Ki for integral constant
	float Kp = 0.41;
	float Ki = 0.3;
	float Kd = 0.09;
	//stores integral values which is calculated by adding error to itself
	float integral = 0;
	float integral2 = 0;
	//stores derivative calculated by
	float derivative = 0;
	float derivative2 = 0;
	//for timer stores time val in MS
	long currentTime;
	//runs until target is reached is essentially a timeout
    for( currentTime = millis(); currentTime > 5000;) {


		//error calculations desired val minus actual
		error = target - encoderGet(encoder);
		error2 = target - encoderGet(encoder2);
		//integral calculations
		integral = integral + error;
		integral2 = integral2 + error2;
		//prevents the integral from getting too big
		if(error == 0)
		{
		integral = 0;
		}

		if(error == 0)
		{
		integral = 0;
		}

		if ( abs(error) > 40)
		{
		integral = 0;
		}

		if ( abs(error2) > 40)
		{
		integral2 = 0;
		}
		derivative = error - preverror;
		derivative2 = error2 - preverror2;
		//assignment of speed values through calculations
		speed = Kp * error + Ki*integral + Kd*derivative;
		speed2 = Kp * error2 + Ki*integral2+ Kd*derivative2;

		motorSet(2,  speed);
		motorSet(3,  speed);
		motorSet(4,  speed2);
		motorSet(5,  speed2);

}
void autonomous() {
	pidcontrol(520);

}
}
