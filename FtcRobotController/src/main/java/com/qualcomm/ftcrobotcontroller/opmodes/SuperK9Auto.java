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
public class SuperK9Auto extends SuperK9Base {

    private static final double RUN_POWER  = 0.25;
    private static final double TURN_POWER = 0.25;
    private static final double TARGET_DISTANCE = 48;

    private enum States {
        RESET_ENCODERS,
        DRIVE_FORWARD,
        TURN_RIGHT,
        STOP
    }

    private States _state;
    private double _targetDistance;

    @Override
    protected void k9Init() {
        this.setHasRearEncoders(true);
    }

    @Override
    protected void k9Start() {
        _state = States.RESET_ENCODERS;
    }

    @Override
    protected void k9Loop() {

        switch(_state) {
            case RESET_ENCODERS:
                this.resetEncoders();
                this.setPower(0, 0);
                if(this.areEncodersReset()) {
                    _targetDistance = TARGET_DISTANCE;
                    _state = States.DRIVE_FORWARD;
                }
                break;
            case DRIVE_FORWARD:
                this.runWithEncoders();
                this.setPower(RUN_POWER, RUN_POWER);
                if(this.getLeftPositionInches() >= _targetDistance && this.getRightPositionInches() >= _targetDistance) {
                    _state = States.STOP;
                    this.setPower(0, 0);
                }
                break;
            case TURN_RIGHT:
                this.runWithEncoders();
                this.setPower(TURN_POWER, -TURN_POWER);
                if(this.getLeftPositionInches() > _targetDistance && this.getRightPositionInches() <= _targetDistance) {
                    _state = States.STOP;
                    this.setPower(0, 0);
                }
            case STOP:
                break;
        }

    }
}
