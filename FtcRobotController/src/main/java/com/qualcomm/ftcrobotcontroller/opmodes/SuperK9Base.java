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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public abstract class SuperK9Base extends OpMode {

    // dead reckoning information //
	final static int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
	final static int    WHEEL_DIAMETER        = 6;    // inches / REV
	final static double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    // servo position information //
    final static double BUTTON_SERVO_POS_LEFT   = 0.34;
    final static double BUTTON_SERVO_POS_CENTER = 0.54;
    final static double BUTTON_SERVO_POS_RIGHT  = 0.74;

    protected enum ButtonServoPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    final static double MAN_SERVO_POS_PARK   = 0.0;
    final static double MAN_SERVO_POS_HOME   = 0.18;
    final static double MAN_SERVO_POS_DEPLOY = 0.78;

    protected enum ManServoPosition {
        PARK,
        HOME,
        DEPLOY
    }

    final static double PLOW_POWER_MIN = -0.25;
    final static double PLOW_POWER_MAX = 0.5;

    final static double DOZER_POWER_MIN = -0.35;
    final static double DOZER_POWER_MAX = 0.5;

    // color sensor information //
    final static int COLOR_LED_CHANNEL = 5;

    final static double HUE_THRESHOLD_RED  = 25.0;
    final static double HUE_THRESHOLD_BLUE = 200.0;

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

    Servo _buttonServo;
    Servo _manServo;
    Servo _plowMotor;
    Servo _dozerMotor;

	DeviceInterfaceModule _cdim;
	ColorSensor _sensorRGB;
	OpticalDistanceSensor _sensorODS;
	LightSensor _sensorLego;
    final ElapsedTime _time = new ElapsedTime();

    boolean _hasRearEncoders = false;
    int _leftEncoderOffset  = 0;
    int _rightEncoderOffset = 0;

    protected enum TeamNumber {
        TEAM_8740,
        TEAM_8741
    }
    private TeamNumber _teamNumber;

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
		_cdim.setDigitalChannelMode(COLOR_LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
		this.setColorSensorLED(false);

		_sensorODS = hardwareMap.opticalDistanceSensor.get("ods");
		_sensorLego = hardwareMap.lightSensor.get("light");

        _buttonServo = hardwareMap.servo.get("buttonServo");
        _buttonServo.setDirection(Servo.Direction.REVERSE);
        this.setButtonServoPosition(ButtonServoPosition.CENTER);

        _manServo = hardwareMap.servo.get("manServo");
        this.setManServoPosition(ManServoPosition.HOME);

        _plowMotor = hardwareMap.servo.get("plowMotor");
        this.setPlowPower(0);

        _dozerMotor = hardwareMap.servo.get("dozerMotor");
        this.setDozerPower(0);

        try {
            DigitalChannel jumper = hardwareMap.digitalChannel.get("jumper");
            _teamNumber = jumper.getState()? TeamNumber.TEAM_8740: TeamNumber.TEAM_8741;
        } catch(Exception e) {
            // no jumper set //
            _teamNumber = TeamNumber.TEAM_8740;
        }
        this.setHasRearEncoders(true);
        this.k9Init();
	}

    /*
     * This method will be called on start
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void start() {
        _time.reset();
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
        telemetry.addData("Left encoder", String.format("%.2f", this.getLeftPositionInches()));
        telemetry.addData("Right encoder", String.format("%.2f", this.getRightPositionInches()));
        telemetry.addData("Plow power", String.format("%.2f", this.getPlowPower()));
        telemetry.addData("Dozer power", String.format("%.2f", this.getDozerPower()));
        telemetry.addData("Team #", this.getTeamNumber());
        //telemetry.addData("Color (Hue)", String.format("%s (%.2f)", this.getColorSensor(), this.getColorSensorHue()));
        //telemetry.addData("ODS", String.format("%.2f", this.getODSLight()));
        //telemetry.addData("Lego", String.format("%.2f", this.getLegoLight()));
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

    protected TeamNumber getTeamNumber() {
        return _teamNumber;
    }

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

    protected ButtonServoPosition getButtonServoPosition() {
        double pos = _buttonServo.getPosition();
        return  pos == BUTTON_SERVO_POS_LEFT? ButtonServoPosition.LEFT:
                pos == BUTTON_SERVO_POS_CENTER? ButtonServoPosition.CENTER:
                        ButtonServoPosition.RIGHT;
    }

    protected void setButtonServoPosition(ButtonServoPosition pos) {
        switch (pos) {
            case LEFT:
                _buttonServo.setPosition(BUTTON_SERVO_POS_LEFT);
                break;
            case CENTER:
                _buttonServo.setPosition(BUTTON_SERVO_POS_CENTER);
                break;
            case RIGHT:
                _buttonServo.setPosition(BUTTON_SERVO_POS_RIGHT);
                break;
        }
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

    protected double getPlowPower() {
        return (_plowMotor.getPosition() * 2) - 1;
    }

    // negative is deploy, positive retract //
    protected void setPlowPower(double power) {
        power = (Range.clip(power, PLOW_POWER_MIN, PLOW_POWER_MAX) + 1) / 2;
        _plowMotor.setPosition(power);
    }

    protected double getDozerPower() {
        return (_dozerMotor.getPosition() * 2) - 1;
    }

    // positive is deploy, negative retract //
    protected void setDozerPower(double power) {
        power = (Range.clip(power, DOZER_POWER_MIN, DOZER_POWER_MAX) + 1) / 2;
        _dozerMotor.setPosition(power);
    }

    protected float getColorSensorHue() {
        float hsvValues[] = {0F,0F,0F};

        Color.RGBToHSV((_sensorRGB.red() * 255) / 800, (_sensorRGB.green() * 255) / 800, (_sensorRGB.blue() * 255) / 800, hsvValues);
        return hsvValues[0];
    }

    protected FtcColor getColorSensor() {
        float hue = this.getColorSensorHue();
        return hue <= HUE_THRESHOLD_RED? FtcColor.RED: hue >= HUE_THRESHOLD_BLUE? FtcColor.BLUE: FtcColor.UNKNOWN;
    }

    protected boolean getColorSensorLED() {
        return _cdim.getDigitalChannelState(COLOR_LED_CHANNEL);
    }

    protected void setColorSensorLED(boolean enabled) {
        _cdim.setDigitalChannelState(COLOR_LED_CHANNEL, enabled);
    }

    protected double getODSLight() {
        return _sensorODS.getLightDetected();
    }

    protected double getLegoLight() {
        return _sensorLego.getLightDetected();
    }

    protected int sign(double value) {
        return value > 0? 1: value < 0? -1: 0;
    }

    private void setEncoderMode(DcMotorController.RunMode mode) {
        _motorLeftFront.setMode(mode);
        _motorRightFront.setMode(mode);

        if(this.hasRearEncoders()) {
            _motorLeftRear.setMode(mode);
            _motorRightRear.setMode(mode);
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

    /**
     *  Auto Command inner state machine. Use these for actions that require continually testing
     *  sensors or persistent state between loop invocations.
     */
    private enum AutoCommandState {
        NONE,
        DRIVE,
        TURN,
        WAIT
    }
    private AutoCommandState _commandState = AutoCommandState.NONE;

    protected void autoEnd() {
        _commandState = AutoCommandState.NONE;
    }

    protected boolean autoDriveDistance(double inches, double speed) {
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.setPower(0, 0);
                _commandState = AutoCommandState.DRIVE;
                break;
            case DRIVE:
                this.runWithEncoders();
                this.setPower(this.sign(inches) * speed, this.sign(inches) * speed);
                // check if we are at target distance //
                double left  = this.getLeftPositionInches();
                double right = this.getRightPositionInches();
                if(  (inches >= 0 && left >= inches && right >= inches)
                        || (inches < 0  && left <= inches && right <= inches))
                {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("drive called without ending previous command: " + _commandState);
        }
        return false;
    }

    // negative is clockwise, positive is counterclockwise //
    protected boolean autoTurnPivot(double inches, double speed) {
        if(speed < 0) throw new IllegalArgumentException("speed: " + speed);
        speed = Range.clip(speed, 0, 1.0);
        switch(_commandState) {
            case NONE:
                this.resetEncoders();
                this.setPower(0, 0);
                _commandState = AutoCommandState.TURN;
                break;
            case TURN:
                this.runWithEncoders();
                double current = 0;
                if(inches >= 0) {
                    this.setPower(0, speed);
                    current = this.getRightPositionInches();
                } else {
                    this.setPower(speed, 0);
                    current = this.getLeftPositionInches();
                }
                if(current >= inches) {
                    this.setPower(0, 0);
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("turn called without ending previous command: " + _commandState);
        }
        return false;
    }

    // wait for a specific number of seconds //
    private double _autoWaitStart = 0;
    protected boolean autoWaitSeconds(double seconds) {
        if(seconds < 0) throw new IllegalArgumentException("seconds: " + seconds);
        switch(_commandState) {
            case NONE:
                _autoWaitStart = _time.time();
                _commandState  = AutoCommandState.WAIT;
                break;
            case WAIT:
                if(_time.time() > _autoWaitStart + seconds) {
                    _commandState = AutoCommandState.NONE;
                    return true;
                }
                break;
            default:
                throw new IllegalStateException("wait called without ending previous command: " + _commandState);
        }
        return false;
    }

}
