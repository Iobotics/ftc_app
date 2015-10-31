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

import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class SuperK9Auto2 extends SuperK9Base {

    private static final double RUN_POWER  = 0.25;
    private static final double TURN_POWER = 0.50;

    private enum States {
        START,
        DRIVE_TO_BEACON_ZONE,
        TURN_TO_BEACON,
        DRIVE_TO_BEACON,
        WAIT_FOR_TIME,
        DEPLOY_MAN_DROPPER,
        RESET_MAN_DROPPER,
        READ_COLOR_SENSOR,
        REVERSE_FOR_PUSH,
        SET_BUTTON_PUSHER,
        PUSH_BUTTON,
        LEAVE_BEACON,
        LOWER_PLOW,
        RAISE_PLOW,
        LOWER_DOZER,
        RAISE_DOZER,
        STOP
    }

    // change this to enable beacon pushing //
    private static boolean _doBeacon = false;

    private States _state;
    private States _nextState;
    private double _targetValue;
    private double _nextTarget;
    private final ElapsedTime _time = new ElapsedTime();
    private final FtcColor _robotColor;
    private FtcColor _sensorColor;

    public SuperK9Auto2(FtcColor robotColor) {
        _robotColor = robotColor;
    }

    @Override
    protected void k9Init() { }

    @Override
    protected void k9Start() {
        _state = States.START;
        _time.reset();
    }

    @Override
    protected void k9Loop() {
        telemetry.addData("State", _state.name());
        switch(_state) {
            case START:
                this.resetEncoders();
                this.setPower(0, 0);

                _state = States.DRIVE_TO_BEACON_ZONE;
                _targetValue = 86.5;
                break;
            case DRIVE_TO_BEACON_ZONE:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetValue && this.getRightPositionInches() >= _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, turn right //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.TURN_TO_BEACON;
                    _nextTarget  = 15;
                }
                break;
            case TURN_TO_BEACON:
                this.runWithEncoders();
                // blue -> turn right, red -> turn left //
                if(_robotColor == FtcColor.BLUE) {
                    // run left wheel, read left encoder //
                    this.setPower(TURN_POWER, 0);
                    _nextTarget = this.getLeftPositionInches();
                } else {
                    // run right wheel, read right encoder //
                    this.setPower(0, TURN_POWER);
                    _nextTarget = this.getRightPositionInches();
                }
                if(_nextTarget > _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, drive forward //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.DRIVE_TO_BEACON;
                    _nextTarget  = 5;
                }
                break;
            case DRIVE_TO_BEACON:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetValue && this.getRightPositionInches() >= _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, deploy man //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.DEPLOY_MAN_DROPPER;
                }
                break;
            case DEPLOY_MAN_DROPPER:
                this.setManServoPosition(ManServoPosition.DEPLOY);

                // wait for 2 seconds, reset man //
                _state = States.WAIT_FOR_TIME;
                _targetValue = _time.time() + 2;
                _nextState   = States.RESET_MAN_DROPPER;
                break;
            case RESET_MAN_DROPPER:
                this.setManServoPosition(ManServoPosition.HOME);
                if(_doBeacon) {
                    _state = States.READ_COLOR_SENSOR;
                } else {
                    _state = States.LEAVE_BEACON;
                    _nextTarget = 5; // fixme: this is a repeated constant //
                }
                break;
            case READ_COLOR_SENSOR:
                _sensorColor = this.getColorSensor();

                _state       = States.REVERSE_FOR_PUSH;
                _targetValue = 5;
                break;
            case REVERSE_FOR_PUSH:
                this.runWithEncoders();
                this.setPower(-RUN_POWER, -RUN_POWER);
                if(this.getLeftPositionInches() <= -_targetValue && this.getRightPositionInches() <= -_targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    _state = States.SET_BUTTON_PUSHER;
                }
                break;
            case SET_BUTTON_PUSHER:
                // sensor senses the right half of the beacon //
                // if we see our color, choose the right side, otherwise choose the left //
                this.setButtonServoPosition(_sensorColor == _robotColor? ButtonServoPosition.RIGHT: ButtonServoPosition.LEFT);

                // wait for half a second, drive forward //
                _state = States.WAIT_FOR_TIME;
                _targetValue = _time.time() + 0.5;
                _nextState   = States.PUSH_BUTTON;
                _nextTarget  = 5;
                break;
            case PUSH_BUTTON:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetValue && this.getRightPositionInches() >= _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, reverse //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.LEAVE_BEACON;
                    _nextTarget  = 5;
                }
                break;
            case LEAVE_BEACON:
                this.runWithEncoders();
                this.setPower(-RUN_POWER, -RUN_POWER);
                if(this.getLeftPositionInches() <= -_targetValue && this.getRightPositionInches() <= -_targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    _state = States.STOP;
                }
                break;
            case LOWER_PLOW:
                if(this.getPlowPower() == 0) {
                    this.setPlowPower(-1.0);
                    _targetValue = _time.time() + 1.5;
                    break;
                }
                if(_time.time() >= _targetValue) {
                    this.setPlowPower(0);
                    _state = States.STOP; // fixme //
                }
                break;
            case RAISE_PLOW:
                if(this.getPlowPower() == 0) {
                    this.setPlowPower(1.0);
                    _targetValue = _time.time() + 1.5;
                    break;
                }
                if(_time.time() >= _targetValue) {
                    this.setPlowPower(0);
                    _state = States.STOP; // fixme //
                }
                break;
            case LOWER_DOZER:
                if(this.getDozerPower() == 0) {
                    this.setDozerPower(-1.0);
                    _targetValue = _time.time() + 1.5;
                    break;
                }
                if(_time.time() >= _targetValue) {
                    this.setDozerPower(0);
                    _state = States.STOP; // fixme //
                }
                break;
            case RAISE_DOZER:
                if(this.getDozerPower() == 0) {
                    this.setDozerPower(1.0);
                    _targetValue = _time.time() + 1.5;
                    break;
                }
                if(_time.time() >= _targetValue) {
                    this.setDozerPower(0);
                    _state = States.STOP; // fixme //
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
                this.setButtonServoPosition(ButtonServoPosition.CENTER);
                //this.runWithoutEncoders();
                break;
        }

    }
}
