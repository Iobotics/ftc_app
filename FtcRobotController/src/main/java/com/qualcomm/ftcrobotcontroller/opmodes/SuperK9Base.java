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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public abstract class SuperK9Base extends OpMode {

	//final static int LED_CHANNEL = 5;
	//final static boolean LED_ENABLED = false;

	final static int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
	final static int    WHEEL_DIAMETER        = 6; // inches / REV
	final static double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

	final static int LED_CHANNEL = 5;

    final static double SERVO_POS_LEFT   = 0.0;
    final static double SERVO_POS_CENTER = 0.5;
    final static double SERVO_POS_RIGHT  = 1.0;

	DcMotor _motorRightFront;
	DcMotor _motorRightRear;
	DcMotor _motorLeftFront;
	DcMotor _motorLeftRear;

    Servo _servo;

	DeviceInterfaceModule _cdim;
	ColorSensor _sensorRGB;
	OpticalDistanceSensor _sensorODS;
	LightSensor _sensorLego;

    protected enum ServoPosition {
        LEFT,
        CENTER,
        RIGHT
    }

	/*
	 * Code to run when the op mode is initialized goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
	 */
	@Override
	public void init() {
		_motorRightFront = hardwareMap.dcMotor.get("rightFront");
		_motorRightRear  = hardwareMap.dcMotor.get("rightRear");
		_motorLeftFront = hardwareMap.dcMotor.get("leftFront");
		_motorLeftRear = hardwareMap.dcMotor.get("leftRear");

		_motorRightFront.setDirection(DcMotor.Direction.REVERSE);
		_motorRightRear.setDirection(DcMotor.Direction.REVERSE);

		_cdim = hardwareMap.deviceInterfaceModule.get("dim");
		_sensorRGB = hardwareMap.colorSensor.get("color");
		_cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
		this.setColorSensorLED(false);

		_sensorODS = hardwareMap.opticalDistanceSensor.get("ods");
		_sensorLego = hardwareMap.lightSensor.get("light");

        _servo = hardwareMap.servo.get("buttonServo");
        _servo.setDirection(Servo.Direction.REVERSE);
        this.setServoPosition(ServoPosition.CENTER);

        this.k9Init();
	}

    /*
     * This method will be called on start
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void start() {
        this.k9Start();
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        telemetry.addData("Text", "*** Robot Data***");
        this.k9Loop();

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("left encoder", String.format("%.2f", this.getLeftPositionInches()));
        telemetry.addData("right encoder", String.format("%.2f", this.getRightPositionInches()));
        telemetry.addData("Hue", String.format("%.2f", this.getColorSensorHue()));
        telemetry.addData("ODS", String.format("%.2f", this.getODSLight()));
        telemetry.addData("Lego", String.format("%.2f", this.getLegoLight()));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        this.k9Stop();
    }

    protected void k9Init() { }

    protected void k9Start() { }

    protected void k9Loop() { }

    protected void k9Stop() { }

	protected void setPowerScaled(double leftPower, double rightPower) {

        rightPower = Range.clip(rightPower, -1, 1);
        leftPower  = Range.clip(leftPower, -1, 1);

        rightPower = scaleInput(rightPower);
        leftPower =  scaleInput(leftPower);

        this.setPower(leftPower, rightPower);
	}

    protected void setPower(double leftPower, double rightPower) {
        // write the values to the motors
        _motorRightFront.setPower(rightPower);
        _motorRightRear.setPower(rightPower);
        _motorLeftFront.setPower(leftPower);
        _motorLeftRear.setPower(leftPower);
    }

    protected int getLeftEncoder() {
        return _motorLeftFront.getCurrentPosition();
    }

    protected double getLeftPositionInches() {
        return this.getLeftEncoder() * INCHES_PER_TICK;
    }

    protected int getRightEncoder() {
        return _motorRightFront.getCurrentPosition();
    }

    protected double getRightPositionInches() {
        return this.getRightEncoder() * INCHES_PER_TICK;
    }

    protected void resetEncoders() {
        _motorLeftFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        _motorRightFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    protected void runWithEncoders() {
        _motorLeftFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        _motorRightFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    protected void runWithoutEncoders() {
        _motorLeftFront.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        _motorRightFront.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    protected ServoPosition getServoPosition() {
        double pos = _servo.getPosition();
        return  pos == SERVO_POS_LEFT? ServoPosition.LEFT:
                pos == SERVO_POS_CENTER? ServoPosition.CENTER:
                        ServoPosition.RIGHT;
    }

    protected void setServoPosition(ServoPosition pos) {
        switch (pos) {
            case LEFT:
                _servo.setPosition(SERVO_POS_LEFT);
                break;
            case CENTER:
                _servo.setPosition(SERVO_POS_CENTER);
                break;
            case RIGHT:
                _servo.setPosition(SERVO_POS_RIGHT);
                break;
        }
    }

    protected float getColorSensorHue() {
        float hsvValues[] = {0F,0F,0F};

        Color.RGBToHSV((_sensorRGB.red() * 255) / 800, (_sensorRGB.green() * 255) / 800, (_sensorRGB.blue() * 255) / 800, hsvValues);
        return hsvValues[0];
    }

    protected boolean getColorSensorLED() {
        return _cdim.getDigitalChannelState(LED_CHANNEL);
    }

    protected void setColorSensorLED(boolean enabled) {
        _cdim.setDigitalChannelState(LED_CHANNEL, enabled);
    }

    protected double getODSLight() {
        return _sensorODS.getLightDetected();
    }

    protected double getLegoLight() {
        return _sensorLego.getLightDetected();
    }

	private double scaleInput(double dVal)  {
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
