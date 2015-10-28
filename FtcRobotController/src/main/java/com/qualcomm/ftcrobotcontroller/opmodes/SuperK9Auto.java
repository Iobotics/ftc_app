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
        DRIVE_FORWARD,
        TURN_RIGHT,
        DRIVE_FORWARD2,
        WAIT_FOR_TIME,
        DEPLOY_MAN,
        RESET_MAN,
        DRIVE_REVERSE,
        SET_BUTTON_PUSHER,
        DRIVE_FORWARD3,
        DRIVE_REVERSE2,
        STOP
    }

    private States _state;
    private States _nextState;
    private double _targetValue;
    private double _nextTarget;
    private final ElapsedTime _time = new ElapsedTime();
    private FtcColor _color;

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

                _state = States.DRIVE_FORWARD;
                _targetValue = 86.5;
                break;
            case DRIVE_FORWARD:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetValue && this.getRightPositionInches() >= _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, turn right //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.TURN_RIGHT;
                    _nextTarget  = 15;
                }
                break;
            case TURN_RIGHT:
                this.runWithEncoders();
                this.setPower(TURN_POWER, 0);
                if(this.getLeftPositionInches() > _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, drive forward //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.DRIVE_FORWARD2;
                    _nextTarget  = 5;
                }
                break;
            case DRIVE_FORWARD2:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetValue && this.getRightPositionInches() >= _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, deploy man //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.DEPLOY_MAN;
                }
                break;
            case DEPLOY_MAN:
                this.setManServoPosition(ManServoPosition.DEPLOY);

                // wait for 2 seconds, reset man //
                _state = States.WAIT_FOR_TIME;
                _targetValue = _time.time() + 2;
                _nextState   = States.RESET_MAN;
                break;
            case RESET_MAN:
                this.setManServoPosition(ManServoPosition.HOME);
                _color = this.getColorSensor();

                _state       = States.DRIVE_REVERSE;
                _targetValue = 5;
                break;
            case DRIVE_REVERSE:
                this.runWithEncoders();
                this.setPower(-RUN_POWER, -RUN_POWER);
                if(this.getLeftPositionInches() <= -_targetValue && this.getRightPositionInches() <= -_targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    _state = States.SET_BUTTON_PUSHER;
                }
                break;
            case SET_BUTTON_PUSHER:
                this.setButtonServoPosition(_color == FtcColor.RED? ButtonServoPosition.LEFT: ButtonServoPosition.RIGHT);

                // wait for half a second, drive forward //
                _state = States.WAIT_FOR_TIME;
                _targetValue = _time.time() + 0.5;
                _nextState   = States.DRIVE_FORWARD3;
                _nextTarget  = 5;
                break;
            case DRIVE_FORWARD3:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetValue && this.getRightPositionInches() >= _targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    // wait for 1 second, reverse //
                    _state = States.WAIT_FOR_TIME;
                    _targetValue = _time.time() + 1;
                    _nextState   = States.DRIVE_REVERSE2;
                    _nextTarget  = 5;
                }
                break;
            case DRIVE_REVERSE2:
                this.runWithEncoders();
                this.setPower(-RUN_POWER, -RUN_POWER);
                if(this.getLeftPositionInches() <= -_targetValue && this.getRightPositionInches() <= -_targetValue) {
                    this.setPower(0, 0);
                    this.resetEncoders();

                    _state = States.STOP;
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
