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
public abstract class TankbotBase extends OpMode {

    // dead reckoning information //
	final static int    ENCODER_TICKS_PER_REV = 560; // Neverest 20
	final static int    WHEEL_DIAMETER        = 2;    // inches / REV
	final static double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    final static double MAN_SERVO_POS_PARK   = 0.9;
    final static double MAN_SERVO_POS_HOME   = 0.7;
    final static double MAN_SERVO_POS_DEPLOY = 0.3;

    protected enum ManServoPosition {
        PARK,
        HOME,
        DEPLOY
    }

    final static double TRIGGER_LEFT_POS_IN  = 0.5;
    final static double TRIGGER_LEFT_POS_OUT = 0.03;

    final static double TRIGGER_RIGHT_POS_IN  = 0.03;
    final static double TRIGGER_RIGHT_POS_OUT = 0.5;

    protected enum FtcColor {
        RED,
        BLUE,
        UNKNOWN
    }

    // hardware instances //
	DcMotor _motorRightFront;
	DcMotor _motorRightRear;
	DcMotor _motorLeftFront;
	DcMotor _motorLeftRear;

    Servo _manServo;
    Servo _leftTrigger;
    Servo _rightTrigger;

    boolean _hasRearEncoders = false;
    int _leftEncoderOffset  = 0;
    int _rightEncoderOffset = 0;

	/*
	 * Code to run when the op mode is initialized goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
	 */
	@Override
	public final void init() {
        _motorRightFront = hardwareMap.dcMotor.get("rightFront");
        _motorRightRear  = hardwareMap.dcMotor.get("rightRear");
        _motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        _motorLeftRear = hardwareMap.dcMotor.get("leftRear");

        _motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        _motorLeftRear.setDirection(DcMotor.Direction.REVERSE);

        _manServo = hardwareMap.servo.get("manServo");
        this.setManServoPosition(ManServoPosition.HOME);

        _rightTrigger = hardwareMap.servo.get("rightTrigger");
        this.setRightTriggerDeployed(false);
        _leftTrigger  = hardwareMap.servo.get("leftTrigger");
        this.setLeftTriggerDeployed(false);

        _rightTrigger.setDirection(Servo.Direction.REVERSE);

        this.setHasRearEncoders(true);
        this.TBInit();
	}

    /*
     * This method will be called on start
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public final void start() {
        this.TBStart();
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public final void loop() {
        telemetry.addData("Text", "*** Robot Data***");
        this.TBLoop();

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        //telemetry.addData("Left encoder", String.format("%.2f", this.getLeftPositionInches()));
        //telemetry.addData("Right encoder", String.format("%.2f", this.getRightPositionInches()));
        telemetry.addData("Man Servo", this.getManServoPosition());
        telemetry.addData("Left Trigger", this.getLeftTriggerDeployed());
        telemetry.addData("Right Trigger", this.getRightTriggerDeployed());
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public final void stop() {
        this.TBStop();
    }

    protected void TBInit() { }

    protected void TBStart() { }

    protected void TBLoop() { }

    protected void TBStop() { }


    protected boolean getLeftTriggerDeployed() {
        return _leftTrigger.getPosition() == TRIGGER_LEFT_POS_OUT;
    }

    protected void setLeftTriggerDeployed(boolean out) {
        _leftTrigger.setPosition(out ? TRIGGER_LEFT_POS_OUT: TRIGGER_LEFT_POS_IN);
    }

    protected boolean getRightTriggerDeployed() {
        return _rightTrigger.getPosition() == TRIGGER_RIGHT_POS_OUT;
    }

    protected void setRightTriggerDeployed(boolean out) {
        _rightTrigger.setPosition(out ? TRIGGER_RIGHT_POS_OUT: TRIGGER_RIGHT_POS_IN);
    }

	protected void setPowerScaled(double leftPower, double rightPower) {

        rightPower = Range.clip(rightPower, -1, 1);
        leftPower  = Range.clip(leftPower, -1, 1);

        rightPower = scaleInput(rightPower);
        leftPower = scaleInput(leftPower);

        this.setPower(leftPower, rightPower);
	}

    protected void setPower(double leftPower, double rightPower) {
        // write the values to the motors
        _motorRightFront.setPower(rightPower);
        _motorRightRear.setPower(rightPower);
        _motorLeftFront.setPower(leftPower);
        _motorLeftRear.setPower(leftPower);
    }

    protected boolean hasRearEncoders() {
        return _hasRearEncoders;
    }

    protected void setHasRearEncoders(boolean has) {
        _hasRearEncoders = has;
    }

    protected int getLeftEncoder() {
        return _motorLeftFront.getCurrentPosition() - _leftEncoderOffset;
    }

    protected double getLeftPositionInches() {
        return this.getLeftEncoder() * INCHES_PER_TICK;
    }

    protected int getRightEncoder() {
        return _motorRightFront.getCurrentPosition() - _rightEncoderOffset;
    }

    protected double getRightPositionInches() {
        return this.getRightEncoder() * INCHES_PER_TICK;
    }

    protected void resetEncoders() {
        //this.setEncoderMode(DcMotorController.RunMode.RESET_ENCODERS);
        _leftEncoderOffset  = _motorLeftFront.getCurrentPosition();
        _rightEncoderOffset = _motorRightFront.getCurrentPosition();
    }

    protected boolean areEncodersReset() {
        return this.getLeftEncoder() == 0 && this.getRightEncoder() == 0;
    }

    protected void runWithEncoders() {
        this.setEncoderMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    protected void runWithoutEncoders() {
        this.setEncoderMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    protected void runToPosition(double leftInches, double rightInches, double power) {
        int leftTicks  = (int) ((leftInches  + _leftEncoderOffset)  / INCHES_PER_TICK);
        int rightTicks = (int) ((rightInches + _rightEncoderOffset) / INCHES_PER_TICK);

        this.setEncoderMode(DcMotorController.RunMode.RUN_TO_POSITION);
        _motorLeftFront.setTargetPosition(leftTicks);
        _motorRightFront.setTargetPosition(rightTicks);
        if(this.hasRearEncoders()) {
            _motorLeftRear.setTargetPosition(leftTicks);
            _motorRightRear.setTargetPosition(rightTicks);
        }
        this.setPower(this.sign(leftInches) * power, this.sign(rightInches) * power);
    }

    protected boolean isRunning() {
        return _motorLeftFront.isBusy() || _motorRightFront.isBusy();
    }

    protected ManServoPosition getManServoPosition() {
        double pos = _manServo.getPosition();
        return  pos == MAN_SERVO_POS_DEPLOY? ManServoPosition.DEPLOY:
                pos == MAN_SERVO_POS_HOME? ManServoPosition.HOME:
                        ManServoPosition.PARK;
    }

    protected void setManServoPosition(ManServoPosition pos) {
        switch (pos) {
            case HOME:
                _manServo.setPosition(MAN_SERVO_POS_HOME);
                break;
            case DEPLOY:
                _manServo.setPosition(MAN_SERVO_POS_DEPLOY);
                break;
            case PARK:
                _manServo.setPosition(MAN_SERVO_POS_PARK);
                break;
        }
    }

    private int sign(double value) {
        return value > 0? 1: value < 0? -1: 0;
    }

    private void setEncoderMode(DcMotorController.RunMode mode) {
        _motorLeftFront.setChannelMode(mode);
        _motorRightFront.setChannelMode(mode);

        if(this.hasRearEncoders()) {
            _motorLeftRear.setChannelMode(mode);
            _motorRightRear.setChannelMode(mode);
        }
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

		double dScale;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
	}

}
