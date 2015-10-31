package com.qualcomm.ftcrobotcontroller.opmodes;

//Note: Some inputs may not be needed; delete when finished
import android.net.NetworkInfo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




/**
 * Created by Student on 10/30/2015.
 */
public class TankbotAuto extends OpMode {
    final static int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final static int    WHEEL_DIAMETER        = 2;    // inches / REV... we're using tank treads so this is really a gear diameter
    final static double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    private static final double RUN_POWER  = 0.25;
    private static final double TURN_POWER = 0.50;

    final static double MAN_SERVO_POS_HOME   = 0.0;
    final static double MAN_SERVO_POS_DEPLOY = 1.0;

    // Motors //
    DcMotor _motorRightFront;
    DcMotor _motorRightRear;
    DcMotor _motorLeftFront;
    DcMotor _motorLeftRear;

    DcMotorController wheelControllerLeft;
    DcMotorController wheelControllerRight;

    private enum States {
        START,
        DRIVE_FORWARD,
        TURN_RIGHT,
        DRIVE_FORWARD2,
        WAIT_FOR_TIME,
        DEPLOY_MAN,
        RESET_MAN,
        DRIVE_REVERSE,
        STOP
    }

    //NOTE: We can only write in write-only, and same for read... need to alternate between them when we need to do both
    private enum StateModes { //Is this state in read-only or write-only mode?
        READ,
        WRITE
    }

    private enum ManServoPosition {
        HOME,
        DEPLOY
    }

    private States _state;
    private States _nextState;
    private StateModes _stateMode;

    private double _leftPositionInches;
    private double _rightPositionInches;

    private double _targetValue;
    private double _nextTarget;
    private final ElapsedTime _time = new ElapsedTime();

    Servo _manServo;

    @Override
    public void init() {
        _motorRightFront = hardwareMap.dcMotor.get("rightFront");
        _motorRightRear  = hardwareMap.dcMotor.get("rightRear");
        _motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        _motorLeftRear = hardwareMap.dcMotor.get("leftRear");

        _motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        _motorLeftRear.setDirection(DcMotor.Direction.REVERSE);

        _manServo = hardwareMap.servo.get("manServo");

        this.setManServoPosition(ManServoPosition.HOME);

        wheelControllerLeft = hardwareMap.dcMotorController.get("wheelMotorController");
        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        wheelControllerRight = hardwareMap.dcMotorController.get("wheelMotorController");
        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
    }

    @Override
    public void start() {
        _state = States.START;
        _stateMode = StateModes.READ;
        _time.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("State", _state.name());

        switch(_state) {
            case START:
                this.resetEncoders();
                this.setPower(0, 0);

                _state = States.DRIVE_FORWARD;
                _targetValue = 86.5;
                break;
            case DRIVE_FORWARD:

                switch(_stateMode) {
                    case READ:
                        this._leftPositionInches = this.getLeftPositionInches();
                        this._rightPositionInches = this.getRightPositionInches();

                        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _stateMode = StateModes.WRITE;
                        break;
                    case WRITE:
                        this.setPower(RUN_POWER, RUN_POWER);

                        if (this._leftPositionInches >= _targetValue && this._rightPositionInches >= _targetValue) {
                            this.setPower(0, 0);
                            this.resetEncoders();

                            // wait for 1 second, turn right //
                            _state = States.WAIT_FOR_TIME;
                            _targetValue = _time.time() + 1;
                            _nextState = States.TURN_RIGHT;
                            _nextTarget = 15;
                        }

                        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _stateMode = StateModes.READ;
                        break;
                }
                break;
            case TURN_RIGHT:
                switch(_stateMode){
                    case READ:
                        this._leftPositionInches = this.getLeftPositionInches();

                        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _stateMode = StateModes.WRITE;
                        break;
                    case WRITE:
                        this.setPower(TURN_POWER, 0);
                        if(this._leftPositionInches > _targetValue) {
                            this.setPower(0, 0);
                            this.resetEncoders();

                            // wait for 1 second, drive forward //
                            _state = States.WAIT_FOR_TIME;
                            _targetValue = _time.time() + 1;
                            _nextState   = States.DRIVE_FORWARD2;
                            _nextTarget  = 5;
                        }

                        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _stateMode = StateModes.READ;
                        break;
                }
                break;
            case DRIVE_FORWARD2:
                switch(_stateMode) {
                    case READ:
                        this._leftPositionInches = this.getLeftPositionInches();
                        this._rightPositionInches = this.getRightPositionInches();

                        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _stateMode = StateModes.WRITE;
                        break;
                    case WRITE:
                        this.setPower(RUN_POWER, RUN_POWER);
                        if(this._leftPositionInches >= _targetValue && this._rightPositionInches >= _targetValue) {
                            this.setPower(0, 0);
                            this.resetEncoders();

                            // wait for 1 second, deploy man //
                            _state = States.WAIT_FOR_TIME;
                            _targetValue = _time.time() + 1;
                            _nextState   = States.DEPLOY_MAN;
                        }
                        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _stateMode = StateModes.READ;
                        break;
                }
                break;
            case DEPLOY_MAN:
                _stateMode = StateModes.WRITE;
                wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY); //Putting this here is a messy solution but should work
                wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                this.setManServoPosition(ManServoPosition.DEPLOY);

                // wait for 2 seconds, reset man //
                _state = States.WAIT_FOR_TIME;
                _targetValue = _time.time() + 2;
                _nextState   = States.RESET_MAN;
                break;
            case RESET_MAN:
                this.setManServoPosition(ManServoPosition.HOME);
                _state       = States.DRIVE_REVERSE;
                _targetValue = 5;
                break;
            case DRIVE_REVERSE:
                switch(_stateMode){
                    case READ:
                        this._leftPositionInches = this.getLeftPositionInches();
                        this._rightPositionInches = this.getRightPositionInches();

                        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _stateMode = StateModes.WRITE;
                        break;
                    case WRITE:
                        this.setPower(-RUN_POWER, -RUN_POWER);
                        if(this._leftPositionInches <= -_targetValue && this._rightPositionInches <= -_targetValue) {
                            this.setPower(0, 0);
                            this.resetEncoders();

                            //_state = States.SET_BUTTON_PUSHER;
                            _state = States.STOP; //Cutting out button pusher because of lack of hardware support
                        }

                        wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _stateMode = StateModes.READ;
                        break;
                }
                break;
            // Generic States //
            case WAIT_FOR_TIME:
                if(_time.time() >= _targetValue) {
                    _state       = _nextState;
                    _targetValue = _nextTarget;
                }
                break;
            case STOP:
                this.resetEncoders();
                //this.runWithoutEncoders();
                break;
        }
    }

    private void resetEncoders() {
        _motorLeftFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        _motorRightFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    private void setPower(double powerLeft, double powerRight) {
        _motorLeftFront.setPower(powerLeft);
        _motorLeftRear.setPower(powerLeft);
        _motorRightFront.setPower(powerRight);
        _motorRightRear.setPower(powerRight);
    }

    private int getLeftEncoder() {
        int pos = _motorLeftFront.getCurrentPosition();
        return pos;
    }

    private int getRightEncoder() {
        int pos = _motorRightFront.getCurrentPosition();
        return pos;
    }

    private double getLeftPositionInches() {
        return this.getLeftEncoder() * this.INCHES_PER_TICK;
    }

    private double getRightPositionInches() {
        return this.getRightEncoder() * this.INCHES_PER_TICK;
    }

    private void setManServoPosition(ManServoPosition pos) {
        switch (pos) {
            case HOME:
                _manServo.setPosition(MAN_SERVO_POS_HOME);
                break;
            case DEPLOY:
                _manServo.setPosition(MAN_SERVO_POS_DEPLOY);
                break;
        }
    }
}
