/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public abstract class K9Base extends OpMode {

	//final static int LED_CHANNEL = 5;
	//final static boolean LED_ENABLED = false;

	final static int ENCODER_TICKS_PER_REV = 1120; // Neverest 40
	final static int WHEEL_DIAMETER = 6; // inches
	final static double TICKS_PER_INCH = (WHEEL_DIAMETER * Math.PI)/ENCODER_TICKS_PER_REV;

	final static int LED_CHANNEL = 5;

	DcMotor motorRightFront;
	DcMotor motorRightRear;
	DcMotor motorLeftFront;
	DcMotor motorLeftRear;

	DeviceInterfaceModule cdim;
	ColorSensor sensorRGB;

	float hsvValues[] = {0F,0F,0F};

	OpticalDistanceSensor sensorODS;
	LightSensor sensorLego;


	/**
	 * Constructor
	 */
	public K9Base() {

	}

	/*
	 * Code to run when the op mode is initialized goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
	 */
	@Override
	public void init() {

		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
		motorRightFront = hardwareMap.dcMotor.get("rightFront");
		motorRightRear  = hardwareMap.dcMotor.get("rightRear");
		motorLeftFront = hardwareMap.dcMotor.get("leftFront");
		motorLeftRear = hardwareMap.dcMotor.get("leftRear");

		motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
		motorLeftRear.setDirection(DcMotor.Direction.REVERSE);

		//motorLeftFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
		//motorRightFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);


		cdim = hardwareMap.deviceInterfaceModule.get("dim");
		sensorRGB = hardwareMap.colorSensor.get("color");
		cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
		cdim.setDigitalChannelState(LED_CHANNEL, true);

		sensorODS = hardwareMap.opticalDistanceSensor.get("ods");
		sensorLego = hardwareMap.lightSensor.get("light");

	}

	/*
	 * Autonomous program
	 *
	 * Move forward 2 feet
	 */
	@Override
	public void start() {
		/*
		int distance = 24;
		motorLeftFront.setTargetPosition(TICKS_PER_INCH * distance);
		motorRightFront.setTargetPosition(TICKS_PER_INCH * distance);

		motorLeftFront.setPower(1.0);
		motorLeftRear.setPower(1.0);
		motorRightFront.setPower(1.0);
		motorRightRear.setPower(1.0);

		//while() { }

		motorLeftFront.setPower(0.0);
		motorLeftRear.setPower(0.0);
		motorRightFront.setPower(0.0);
		motorRightRear.setPower(0.0);
		*/
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

		// throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		// direction: left_stick_x ranges from -1 to 1, where -1 is full left
		// and 1 is full right
		/* float throttle = -gamepad1.left_stick_y;
		float direction = gamepad1.left_stick_x;
		float right = throttle - direction;
		float left = throttle + direction; */
		float left = -gamepad1.left_stick_y;
		float right = -gamepad1.right_stick_y;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float)scaleInput(right);
		left =  (float)scaleInput(left);

		// write the values to the motors
		motorRightFront.setPower(right);
		motorRightRear.setPower(right);
		motorLeftFront.setPower(left);
		motorLeftRear.setPower(left);

		Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  String.format("%.2f", left));
        telemetry.addData("right tgt pwr", String.format("%.2f", right));
		telemetry.addData("left encoder", motorLeftFront.getCurrentPosition());
		telemetry.addData("right encoder", motorRightFront.getCurrentPosition());
		telemetry.addData("Hue", hsvValues[0]);
		telemetry.addData("ODS", sensorODS.getLightDetected());
		telemetry.addData("Lego", sensorLego.getLightDetected());

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}

	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	abstract void setMotors(int left, int right);

	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		if (index < 0) {
			index = -index;
		} else if (index > 16) {
			index = 16;
		}

		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
	}

}
