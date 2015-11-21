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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TankbotTeleOp extends TankbotBase {

    @Override
    public void TBStart() {
        this.setManServoPosition(ManServoPosition.PARK);
        this.runWithoutEncoders();
    }

    @Override
    public void TBLoop() {

        /*double left  = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        this.setPowerScaled(left, right);*/

        double power = gamepad1.right_stick_y;
        double turn  = gamepad1.left_stick_x;
        this.setArcade(power, turn, true);

        if(gamepad1.y) {
            this.setManServoPosition(ManServoPosition.DEPLOY);
        } else if(gamepad1.b){
            this.setManServoPosition(ManServoPosition.HOME);
        } else if(gamepad1.a) {
            this.setManServoPosition(ManServoPosition.PARK);
        }

        if(gamepad1.dpad_left) {
            this.setLeftTriggerDeployed(true);
            this.setRightTriggerDeployed(false);
        } else if(gamepad1.dpad_right) {
            this.setRightTriggerDeployed(true);
            this.setLeftTriggerDeployed(false);
        } else if(gamepad1.dpad_up){
            this.setLeftTriggerDeployed(false);
            this.setRightTriggerDeployed(false);
        }

        if(gamepad1.right_trigger > 0.5) {
            this.setRightShieldPosition(1.0);
        } else if(gamepad1.right_bumper) {
            this.setRightShieldPosition(0.0);
        }

        if(gamepad1.left_trigger > 0.5) {
            this.setLeftShieldPosition(1.0);
        } else if(gamepad1.left_bumper) {
            this.setLeftShieldPosition(0.0);
        }
    }

    void setArcade(double moveValue, double rotateValue, boolean squaredInputs) {
        // local variables to hold the computed PWM values for the motors
        double leftMotorOutput;
        double rightMotorOutput;

        moveValue = Range.clip(moveValue, -1.0, 1.0);
        rotateValue = Range.clip(rotateValue, -1.0, 1.0);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorOutput = moveValue - rotateValue;
                rightMotorOutput = Math.max(moveValue, rotateValue);
            } else {
                leftMotorOutput = Math.max(moveValue, -rotateValue);
                rightMotorOutput = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorOutput = -Math.max(-moveValue, rotateValue);
                rightMotorOutput = moveValue + rotateValue;
            } else {
                leftMotorOutput = moveValue - rotateValue;
                rightMotorOutput = -Math.max(-moveValue, -rotateValue);
            }
        }
        this.setPower(leftMotorOutput, rightMotorOutput);
    }
}
