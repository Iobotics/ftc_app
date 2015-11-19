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

    private static final int DISTANCE_TO_CENTER = 6;

    private enum States {
        START,
        LOWER_PLOW,
        //DRIVE_TO_LINE,
        //WAIT_FOR_CENTER,
        //CENTER_ON_LINE,
        //WAIT_FOR_ALIGN,
        //ALIGN_TO_LINE,
        DRIVE_TO_BEACON_ZONE,
        WAIT_FOR_TURN,
        TURN_TO_BEACON,
        WAIT_TO_APPROACH,
        DRIVE_TO_BEACON,
        WAIT_FOR_MAN,
        DEPLOY_MAN_DROPPER,
        RESET_MAN_DROPPER,
        READ_COLOR_SENSOR,
        REVERSE_FOR_PUSH,
        SET_BUTTON_PUSHER,
        PUSH_BUTTON,
        WAIT_FOR_BEACON,
        LEAVE_BEACON,
        STOP
    }

    // change this to enable beacon pushing //
    private static boolean _doBeacon = false;

    private States _state;
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
    }

    @Override
    protected void k9Loop() {
        telemetry.addData("State", _state.name());
        switch(_state) {
            case START:
                _state = States.LOWER_PLOW;
                break;
            case LOWER_PLOW:
                this.setPlowPower(-1.0);
                if(this.autoWaitSeconds(1.5)) {
                    this.setPlowPower(0.0);
                    //_state = States.DRIVE_TO_LINE;
                    _state = States.DRIVE_TO_BEACON_ZONE;
                }
                break;
            /*
            case DRIVE_TO_LINE:
                if(this.autoDriveToLine(RUN_POWER)) {
                    _state = States.WAIT_FOR_CENTER;
                }
                break;
            case WAIT_FOR_CENTER:
                if(this.autoWaitSeconds(1.0)) {
                    _state = States.CENTER_ON_LINE;
                }
                break;
            case CENTER_ON_LINE:
                if(this.autoDriveDistance(DISTANCE_TO_CENTER, RUN_POWER)) {
                    _state = States.WAIT_FOR_ALIGN;
                }
                break;
            case WAIT_FOR_ALIGN:
                if(this.autoWaitSeconds(1.0)) {
                    _state = States.ALIGN_TO_LINE;
                }
                break;
            case ALIGN_TO_LINE:
                if(this.autoAlignToLine(TURN_POWER)) {
                    _state = States.WAIT_TO_APPROACH;
                }
                break;
            */
            case DRIVE_TO_BEACON_ZONE:
                if(this.autoDriveDistance(_autoParams.DistanceToBeaconZone, RUN_POWER)) {
                    _state = States.WAIT_FOR_TURN;
                }
                break;
            case WAIT_FOR_TURN:
                if(this.autoWaitSeconds(1.0)) {
                    _state = States.TURN_TO_BEACON;
                }
                break;
            case TURN_TO_BEACON:
                // turn left or right based on the robot color //
                double beaconPivot = _robotColor == FtcColor.BLUE?
                         _autoParams.TurnToBeaconZone:
                        -_autoParams.TurnToBeaconZone;
                if(this.autoTurnPivot(beaconPivot, TURN_POWER)) {
                    _state = States.WAIT_TO_APPROACH;
                }
                break;
            case WAIT_TO_APPROACH:
                if(this.autoWaitSeconds(1.0)) {
                    _state = States.DRIVE_TO_BEACON;
                }
                break;
            case DRIVE_TO_BEACON:
                if(this.autoDriveDistance(_autoParams.DistanceToBeacon, RUN_POWER)) {
                    _state = States.WAIT_FOR_MAN;
                }
                break;
            case WAIT_FOR_MAN:
                if(this.autoWaitSeconds(1.0)) {
                    _state = States.DEPLOY_MAN_DROPPER;
                }
                break;
            case DEPLOY_MAN_DROPPER:
                this.setManServoPosition(ManServoPosition.DEPLOY);
                if(this.autoWaitSeconds(2.0)) {
                    _state = States.RESET_MAN_DROPPER;
                }
                break;
            case RESET_MAN_DROPPER:
                this.setManServoPosition(ManServoPosition.HOME);
                if(_doBeacon) {
                    _state = States.READ_COLOR_SENSOR;
                } else {
                    _state = States.LEAVE_BEACON;
                }
                break;
            case READ_COLOR_SENSOR:
                _sensorColor = this.getColorSensor();
                _state       = States.REVERSE_FOR_PUSH;
                break;
            case REVERSE_FOR_PUSH:
                if(this.autoDriveDistance(_autoParams.DistanceToBeacon, RUN_POWER)) {
                    _state = States.SET_BUTTON_PUSHER;
                }
                break;
            case SET_BUTTON_PUSHER:
                // sensor senses the right half of the beacon //
                // if we see our color, choose the right side, otherwise choose the left //
                this.setButtonServoPosition(_sensorColor == _robotColor? ButtonServoPosition.RIGHT: ButtonServoPosition.LEFT);
                if(this.autoWaitSeconds(0.5)) {
                    _state = States.PUSH_BUTTON;
                }
                break;
            case PUSH_BUTTON:
                if(this.autoDriveDistance(_autoParams.DistanceToBeacon, RUN_POWER)) {
                    _state = States.WAIT_FOR_BEACON;
                }
            case WAIT_FOR_BEACON:
                if(this.autoWaitSeconds(1.0)) {
                    this.setButtonServoPosition(ButtonServoPosition.CENTER);
                    _state = States.LEAVE_BEACON;
                }
                break;
            case LEAVE_BEACON:
                if(this.autoDriveDistance(-_autoParams.DistanceToLeaveBeacon, RUN_POWER)) {
                    _state = States.STOP;
                }
                break;
            case STOP:
                break;
        }
    }

    @Override
    protected void k9Stop() {
        this.autoEnd();
    }
}
