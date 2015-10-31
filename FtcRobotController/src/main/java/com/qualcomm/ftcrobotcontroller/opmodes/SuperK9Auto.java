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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class SuperK9Auto extends SuperK9Base {

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
        BACKUP_FOR_PUSH,
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
    private AutoParameters _autoParams;

    public static class AutoParameters {
        private AutoParameters() { }
        public double DistanceToBeaconZone;
        public double TurnToBeaconZone;
        public double DistanceToBeacon;
        public double DistanceToLeaveBeacon;
    }

    public static final AutoParameters Auto8740Blue = new AutoParameters();
    public static final AutoParameters Auto8740Red  = new AutoParameters();
    public static final AutoParameters Auto8741Blue = new AutoParameters();
    public static final AutoParameters Auto8741Red  = new AutoParameters();

    static {
        // Robot 8740 Blue //
        Auto8740Blue.DistanceToBeaconZone  = 92.5;
        Auto8740Blue.TurnToBeaconZone      = 20;
        Auto8740Blue.DistanceToBeacon      = 7;
        Auto8740Blue.DistanceToLeaveBeacon = 0;

        // Robot 8740 Red //
        Auto8740Red.DistanceToBeaconZone  = 89.5;
        Auto8740Red.TurnToBeaconZone      = 25;
        Auto8740Red.DistanceToBeacon      = 7;
        Auto8740Red.DistanceToLeaveBeacon = 0;

        // Robot 8741 Blue //
        Auto8741Blue.DistanceToBeaconZone  = 90;
        Auto8741Blue.TurnToBeaconZone      = 20;
        Auto8741Blue.DistanceToBeacon      = 7;
        Auto8741Blue.DistanceToLeaveBeacon = 0;

        // Robot 8741 Red //
        Auto8741Red.DistanceToBeaconZone  = 89;
        Auto8741Red.TurnToBeaconZone      = 23;
        Auto8741Red.DistanceToBeacon      = 7;
        Auto8741Red.DistanceToLeaveBeacon = 0;

    }

    public SuperK9Auto(FtcColor robotColor) {
        _robotColor = robotColor;
    }

    @Override
    protected void k9Init() {
        this.setManServoPosition(ManServoPosition.HOME);

        // set parameters by team //
        switch(this.getTeamNumber()) {
            case TEAM_8740:
                _autoParams = _robotColor == FtcColor.BLUE? Auto8740Blue: Auto8740Red;
                break;
            case TEAM_8741:
                _autoParams = _robotColor == FtcColor.BLUE? Auto8741Blue: Auto8741Red;
                break;
        }
    }

    @Override
    protected void k9Start() {
        _state = States.START;
        _time.reset();
    }

    @Override
    protected void k9Loop() {
        telemetry.addData("State", _state.name() + " " + _targetValue);
        telemetry.addData("Team Color", _robotColor);
        switch(_state) {
            case START:
                this.resetEncoders();
                this.setPower(0, 0);
                this.setPlowPower(0);

                _state = States.LOWER_PLOW;
                _targetValue = 0;
                break;
            case LOWER_PLOW:
                if(_targetValue == 0) {
                    this.setPlowPower(-1.0);
                    _targetValue = _time.time() + 1.25;
                    break;
                }
                if(_time.time() >= _targetValue) {
                    this.setPlowPower(0);
                    _state = States.DRIVE_TO_BEACON_ZONE;
                    _targetValue = _autoParams.DistanceToBeaconZone;
                }
                break;
            case DRIVE_TO_BEACON_ZONE: // reverse //
                this.runWithEncoders();
                this.setPower(-RUN_POWER, -RUN_POWER);
                if(this.getLeftPositionInches() <= -_targetValue && this.getRightPositionInches() <= -_targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, turn right //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.TURN_TO_BEACON;
                    _nextTarget  = _autoParams.TurnToBeaconZone;
                }
                break;
            case TURN_TO_BEACON:
                this.runWithEncoders();
                // blue -> turn right, red -> turn left //
                if(_robotColor == FtcColor.BLUE) {
                    // run right wheel in reverse, read right encoder //
                    this.setPower(0, -TURN_POWER);
                    _nextTarget = -this.getRightPositionInches();
                } else {
                    // run left wheel in reverse, read left encoder //
                    this.setPower(-TURN_POWER, 0);
                    _nextTarget = -this.getLeftPositionInches();
                }
                if(_nextTarget > _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, drive forward //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.DRIVE_TO_BEACON;
                    _nextTarget  = _autoParams.DistanceToBeacon;
                }
                break;
            case DRIVE_TO_BEACON:
                this.runWithEncoders();
                this.setPower(-RUN_POWER, -RUN_POWER); // reverse //
                if(this.getLeftPositionInches() <= -_targetValue && this.getRightPositionInches() <= -_targetValue) {
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
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 2;
                    _nextState = States.LEAVE_BEACON;
                    _nextTarget = _autoParams.DistanceToLeaveBeacon;
                }
                break;
            case READ_COLOR_SENSOR:
                _sensorColor = this.getColorSensor();

                _state       = States.BACKUP_FOR_PUSH;
                _targetValue = _autoParams.DistanceToBeacon;
                break;
            case BACKUP_FOR_PUSH:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetValue && this.getRightPositionInches() >= _targetValue) {
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
                _nextTarget  = _autoParams.DistanceToBeacon;
                break;
            case PUSH_BUTTON:
                this.runWithEncoders();
                this.setPower(-RUN_POWER, -RUN_POWER); // reverse //
                if(this.getLeftPositionInches() <= -_targetValue && this.getRightPositionInches() <= -_targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, reverse //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.LEAVE_BEACON;
                    _nextTarget  = _autoParams.DistanceToLeaveBeacon;
                }
                break;
            case LEAVE_BEACON:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetValue && this.getRightPositionInches() >= _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    _state = States.STOP;
                }
                break;

            case RAISE_PLOW:
                if(_targetValue == 0) {
                    this.setPlowPower(1.0);
                    _targetValue = _time.time() + 1.25;
                    break;
                }
                if(_time.time() >= _targetValue) {
                    this.setPlowPower(0);
                    _state = States.STOP; // fixme //
                }
                break;
            case LOWER_DOZER:
                if(_targetValue == 0) {
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
                if(_targetValue == 0) {
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
