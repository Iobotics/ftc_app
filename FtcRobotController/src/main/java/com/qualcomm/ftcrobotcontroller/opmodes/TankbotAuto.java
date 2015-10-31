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
public class TankbotAuto extends TankbotBase {

    private static final double RUN_POWER  = 0.25;
    private static final double TURN_POWER = 0.50;

    DcMotorController _wheelControllerLeft;
    DcMotorController _wheelControllerRight;

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

    private States _state;
    private States _nextState;
    private StateModes _stateMode;

    private double _leftPositionInches;
    private double _rightPositionInches;

    private double _targetValue;
    private double _nextTarget;
    private final ElapsedTime _time = new ElapsedTime();

    @Override
    public void TBInit() {
        _wheelControllerLeft = hardwareMap.dcMotorController.get("frontMotors");
        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        _wheelControllerRight = hardwareMap.dcMotorController.get("rearMotor");
        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
    }

    @Override
    public void TBStart() {
        _state = States.START;
        _stateMode = StateModes.READ;
        _time.reset();
    }

    @Override
    public void TBLoop() {
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

                        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
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

                        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _stateMode = StateModes.READ;
                        break;
                }
                break;
            case TURN_RIGHT:
                switch(_stateMode){
                    case READ:
                        this._leftPositionInches = this.getLeftPositionInches();

                        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
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

                        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _stateMode = StateModes.READ;
                        break;
                }
                break;
            case DRIVE_FORWARD2:
                switch(_stateMode) {
                    case READ:
                        _leftPositionInches = this.getLeftPositionInches();
                        _rightPositionInches = this.getRightPositionInches();

                        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _stateMode = StateModes.WRITE;
                        break;
                    case WRITE:
                        this.setPower(RUN_POWER, RUN_POWER);
                        if(_leftPositionInches >= _targetValue && _rightPositionInches >= _targetValue) {
                            this.setPower(0, 0);
                            this.resetEncoders();

                            // wait for 1 second, deploy man //
                            _state = States.WAIT_FOR_TIME;
                            _targetValue = _time.time() + 1;
                            _nextState   = States.DEPLOY_MAN;
                        }
                        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _stateMode = StateModes.READ;
                        break;
                }
                break;
            case DEPLOY_MAN:
                _stateMode = StateModes.WRITE;
                _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY); //Putting this here is a messy solution but should work
                _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
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
                        _leftPositionInches = this.getLeftPositionInches();
                        _rightPositionInches = this.getRightPositionInches();

                        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
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

                        _wheelControllerRight.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                        _wheelControllerLeft.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
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
}
