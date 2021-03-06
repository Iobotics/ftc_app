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

//import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class SuperK9Auto4 extends SuperK9Base {

    private static final double RUN_POWER       = 0.25; // Test 0.50 later
    private static final double FAST_RUN_POWER  = 0.75;
    private static final double GYRO_TURN_POWER = 0.25; // 0.25 MAX! No low battery //
    private static final double ALIGN_POWER     = 0.10;

    private static final double PARTNER_WAIT_SECONDS  = 15;
    private static final double PLOW_WAIT_SECONDS     = 10;
    private static final double LOWER_PLOW_SECONDS    = 1.5;
    private static final double RAISE_PLOW_SECONDS    = 1.5;
    private static final double RESET_LIGHT_SECONDS   = 0.5;
    private static final double MOUNTAIN_WAIT_SECONDS = 8.5;

    private static final int INCHES_TO_LEAVE_WALL_OUTER = 10;
    private static final int INCHES_TO_LEAVE_WALL_INNER = 7;
    private static final int DEGREES_TO_TURN_OUTER = 43;
    private static final int DEGREES_TO_TURN_INNER = 29;
    private static final int INCHES_TO_APPROACH_FAST_OUTER = 72;
    private static final int INCHES_TO_APPROACH_FAST_INNER = 50;
    private static final int INCHES_TO_CENTER = 4;
    private static final int INCHES_TO_BEACON = 5;
    private static final int INCHES_TO_LEAVE_BEACON = 46;
    private static final int INCHES_TO_MOUNTAIN = 50;
    private static final int DEGREES_TO_MOUNTAIN = 45;
    private static final int INCHES_UP_MOUNTAIN = 20;

    private static final int SENSOR_OFFSET_8898 = 4;
    private static final int DEGREE_OFFSET_8898 = 0;

    private enum States {
        START,
        WAIT_FOR_CALIBRATE,
        WAIT_FOR_LOWER_PLOW,
        LOWER_PLOW,
        RESET_LIGHT_SENSORS,
        WAIT_FOR_PARTNER,
        LEAVE_WALL_OUTER,
        LEAVE_WALL_INNER,
        TURN_FOR_APPROACH_OUTER,
        TURN_FOR_APPROACH_INNER,
        APPROACH_LINE_FAST_OUTER,
        APPROACH_LINE_FAST_INNER,
        DRIVE_TO_LINE,
        WAIT_FOR_CENTER,
        CENTER_ON_LINE,
        WAIT_FOR_ALIGN,
        ALIGN_TO_LINE,
        WAIT_TO_APPROACH,
        DRIVE_TO_BEACON,
        WAIT_FOR_MAN,
        START_LOWER_DOZER,
        DEPLOY_MAN_DROPPER,
        RESET_MAN_DROPPER,
        HOVER_MAN_DROPPER,
        DEPLOY_MAN_DROPPER2,
        STOP_LOWER_DOZER,
        RESET_MAN_DROPPER2,
        LEAVE_BEACON_FAST,
        TURN_TO_MOUNTAIN,
        WAIT_FOR_MOUNTAIN,
        DRIVE_TO_MOUNTAIN,
        RAISE_PLOW,
        DRIVE_UP_MOUNTAIN,
        STOP
    }

    public enum EndBehavior {
        DO_NOTHING,
        LEAVE_BEACON,
        GO_TO_MOUNTAIN
    }

    private States _state;
    private final FtcColor _robotColor;

    // behavior parameters //
    private boolean _waitForPartner;
    private EndBehavior _endBehavior;

    public SuperK9Auto4(FtcColor robotColor) {
        this(robotColor, false, EndBehavior.DO_NOTHING);
    }

    public SuperK9Auto4(FtcColor robotColor, boolean wait, EndBehavior end) {
        _robotColor     = robotColor;
        _waitForPartner = wait;
        _endBehavior    = end;
    }

    @Override
    protected void k9Init() {
        this.setManServoPosition(ManServoPosition.HOME);
        this.calibrateGyro();
    }

    @Override
    protected void k9Start() {
        this.setInnerLightLEDColor(_robotColor);
        this.setOuterLightLEDColor(_robotColor);
        _state = States.START;
    }

    @Override
    protected void k9Loop() {
        telemetry.addData("State", _state.name());
        switch(_state) {
            case START:
                _state = this.isGyroCalibrating()? States.WAIT_FOR_CALIBRATE:
                        _waitForPartner? States.WAIT_FOR_LOWER_PLOW: States.LOWER_PLOW;
                break;
            case WAIT_FOR_CALIBRATE:
                if(!this.isGyroCalibrating()) {
                    _state = _waitForPartner? States.WAIT_FOR_LOWER_PLOW: States.LOWER_PLOW;
                }
                break;
            case WAIT_FOR_LOWER_PLOW:
                if(this.autoWaitSeconds(PLOW_WAIT_SECONDS)) {
                    _state = States.LOWER_PLOW;
                }
                break;
            case LOWER_PLOW:
                this.setPlowPower(-1.0);
                if(this.autoWaitSeconds(LOWER_PLOW_SECONDS)) {
                    this.setPlowPower(0.0);
                    _state = States.RESET_LIGHT_SENSORS;
                }
                break;
            case RESET_LIGHT_SENSORS:
                if(this.autoWaitSeconds(RESET_LIGHT_SECONDS)) {
                    this.resetLightSensors();
                    // wait for partner or go immediately //
                    _state = _waitForPartner? States.WAIT_FOR_PARTNER: States.LEAVE_WALL_OUTER;
                }
                break;
            case WAIT_FOR_PARTNER:
                if(this.autoWaitSeconds(PARTNER_WAIT_SECONDS - PLOW_WAIT_SECONDS - LOWER_PLOW_SECONDS - RESET_LIGHT_SECONDS)) {
                    _state = States.LEAVE_WALL_INNER;
                }
                break;
            case LEAVE_WALL_OUTER:
                if (this.autoDriveDistance(-INCHES_TO_LEAVE_WALL_OUTER, RUN_POWER)) {
                    _state = States.TURN_FOR_APPROACH_OUTER;
                }
                break;
            case TURN_FOR_APPROACH_OUTER:
                double degreesOuter = DEGREES_TO_TURN_OUTER + ((this.getTeamNumber() == TeamNumber.TEAM_8898)? DEGREE_OFFSET_8898: 0);
                if (this.autoTurnInPlaceGyro(_robotColor == FtcColor.RED ?  degreesOuter: -degreesOuter, GYRO_TURN_POWER)) {
                    _state = States.APPROACH_LINE_FAST_OUTER;
                }
                break;
            case LEAVE_WALL_INNER:
                if (this.autoDriveDistance(-INCHES_TO_LEAVE_WALL_INNER, RUN_POWER)) {
                    _state = States.TURN_FOR_APPROACH_INNER;
                }
                break;
            case TURN_FOR_APPROACH_INNER:
                double degreesInner = DEGREES_TO_TURN_INNER + ((this.getTeamNumber() == TeamNumber.TEAM_8898)? DEGREE_OFFSET_8898: 0);
                if (this.autoTurnInPlaceGyro(_robotColor == FtcColor.RED ?  degreesInner: -degreesInner, GYRO_TURN_POWER)) {
                    _state = States.APPROACH_LINE_FAST_INNER;
                }
                break;
            case APPROACH_LINE_FAST_OUTER:
                if (this.autoDriveDistance(-INCHES_TO_APPROACH_FAST_OUTER, FAST_RUN_POWER)) {
                    _state = States.DRIVE_TO_LINE;
                }
                break;
            case APPROACH_LINE_FAST_INNER:
                if(this.autoDriveDistance(-INCHES_TO_APPROACH_FAST_INNER, FAST_RUN_POWER)) {
                    _state = States.DRIVE_TO_LINE;
                }
                break;
            case DRIVE_TO_LINE:
                if(this.autoDriveToLine(-RUN_POWER)) {
                    _state = States.WAIT_FOR_CENTER;
                }
                break;
            case WAIT_FOR_CENTER:
                if(this.autoWaitSeconds(0.5)) {
                    _state = States.CENTER_ON_LINE;
                }
                break;
            case CENTER_ON_LINE:
                // offset sensor for 8898 //
                double toCenter = INCHES_TO_CENTER + (this.getTeamNumber() == TeamNumber.TEAM_8898? SENSOR_OFFSET_8898: 0);
                if(this.autoDriveDistance(-toCenter, RUN_POWER)) {
                    _state = States.WAIT_FOR_ALIGN;
                }
                break;
            case WAIT_FOR_ALIGN:
                if(this.autoWaitSeconds(0.5)) {
                    _state = States.ALIGN_TO_LINE;
                }
                break;
            case ALIGN_TO_LINE:
                // align in direction based on color //
                if(this.autoTurnToLine(_robotColor == FtcColor.RED ? ALIGN_POWER : -ALIGN_POWER)) {
                    _state = States.WAIT_TO_APPROACH;
                }
                break;
            case WAIT_TO_APPROACH:
                if(this.autoWaitSeconds(1.0)) {
                    _state = States.DRIVE_TO_BEACON;
                }
                break;
            case DRIVE_TO_BEACON:
                if(this.autoDriveDistance(-INCHES_TO_BEACON, RUN_POWER)) {
                    _state = States.WAIT_FOR_MAN;
                }
                break;
            case WAIT_FOR_MAN:
                if(this.autoWaitSeconds(0.5)) {
                    // start lowering the dozer if we're doing things after //
                    _state = _endBehavior != EndBehavior.DO_NOTHING? States.START_LOWER_DOZER: States.DEPLOY_MAN_DROPPER;
                }
                break;
            case START_LOWER_DOZER:
                this.setDozerPower(-1.0);
                _state = States.DEPLOY_MAN_DROPPER;
                break;
            case DEPLOY_MAN_DROPPER:
                this.setManServoPosition(ManServoPosition.DEPLOY);
                if(this.autoWaitSeconds(1.0)) {
                    // if lowering dozer, stop it now //
                    _state = _endBehavior != EndBehavior.DO_NOTHING? States.STOP_LOWER_DOZER: States.RESET_MAN_DROPPER;
                }
                break;
            case STOP_LOWER_DOZER:
                this.setDozerPower(0);
                _state = States.RESET_MAN_DROPPER;
                break;
            case RESET_MAN_DROPPER:
                this.setManServoPosition(ManServoPosition.HOME);
                _state = States.HOVER_MAN_DROPPER;
                break;
            case HOVER_MAN_DROPPER:
                this.setManServoPosition(ManServoPosition.HOVER);
                if(this.autoWaitSeconds(0.5)) {
                    _state = States.DEPLOY_MAN_DROPPER2;
                }
                break;
            case DEPLOY_MAN_DROPPER2:
                this.setManServoPosition(ManServoPosition.DEPLOY);
                if(this.autoWaitSeconds(0.5)) {
                    _state = States.RESET_MAN_DROPPER2;
                }
                break;
            case RESET_MAN_DROPPER2:
                this.setManServoPosition(ManServoPosition.HOME);
                if(this.autoWaitSeconds(1.0)) {
                    // if no end behavior, stop here //
                    _state = _endBehavior == EndBehavior.DO_NOTHING ? States.STOP : States.LEAVE_BEACON_FAST;
                }
                break;
            case LEAVE_BEACON_FAST:
                if (this.autoDriveDistance(INCHES_TO_LEAVE_BEACON, FAST_RUN_POWER)) {
                    // if only leaving the beacon, stop here //
                    _state = _endBehavior == EndBehavior.LEAVE_BEACON? States.STOP: States.WAIT_FOR_MOUNTAIN;
                }
                break;
            case WAIT_FOR_MOUNTAIN:
                if (this.autoWaitSeconds(MOUNTAIN_WAIT_SECONDS)) {
                    _state = States.TURN_TO_MOUNTAIN;
                }
                break;
            case TURN_TO_MOUNTAIN:
                if (this.autoTurnInPlaceGyro(_robotColor == FtcColor.RED ? DEGREES_TO_MOUNTAIN : -DEGREES_TO_MOUNTAIN, GYRO_TURN_POWER)) {
                    _state = States.DRIVE_TO_MOUNTAIN;
                }
                break;
            case DRIVE_TO_MOUNTAIN:
                if(this.autoDriveDistance(-INCHES_TO_MOUNTAIN, RUN_POWER)) {
                    _state = States.RAISE_PLOW;
                }
                break;
            case RAISE_PLOW:
                this.setPlowPower(1.0);
                if(this.autoWaitSeconds(RAISE_PLOW_SECONDS)) {
                    this.setPlowPower(0.0);
                    _state = States.DRIVE_UP_MOUNTAIN;
                }
                break;
            case DRIVE_UP_MOUNTAIN:
                if(this.autoDriveDistance(-INCHES_UP_MOUNTAIN, RUN_POWER)) {
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
        this.setInnerLightLEDColor(FtcColor.NONE);
        this.setOuterLightLEDColor(FtcColor.NONE);
    }

}
