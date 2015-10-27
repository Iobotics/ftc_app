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

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class TankbotTeleOp extends OpMode {

    // dead reckonining information //
    //final static int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    //final static int    WHEEL_DIAMETER        = 6;    // inches / REV
    //final static double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    // hardware instances //
    //DcMotor _motorRightFront;
    //DcMotor _motorRightRear;
    //DcMotor _motorLeftFront;
    //DcMotor _motorLeftRear;

    Servo _leftServo, _rightServo;

    //OpticalDistanceSensor _sensorODS;
    //LightSensor _sensorLego;


    @Override
    public void init() {
        /*_motorRightFront = hardwareMap.dcMotor.get("rightFront");
        _motorRightRear  = hardwareMap.dcMotor.get("rightRear");
        _motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        _motorLeftRear = hardwareMap.dcMotor.get("leftRear");

        _motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        _motorRightRear.setDirection(DcMotor.Direction.REVERSE);

        _sensorODS = hardwareMap.opticalDistanceSensor.get("ods");
        _sensorLego = hardwareMap.lightSensor.get("light");

        _manServo = hardwareMap.servo.get("manServo");*/
        _leftServo = hardwareMap.servo.get("leftMotor");
        _rightServo = hardwareMap.servo.get("rightMotor");
        _rightServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void start() {
        //this.setEncoderMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

    }

    @Override
    public void loop() {

        /*float pos = (gamepad1.left_stick_y + 1) / 2;
        _manServo.setPosition(pos);
        telemetry.addData("ServoPos", pos);*/
        double left = (-gamepad1.left_stick_y*0.79 + 1) / 2;
        double right = (-gamepad1.right_stick_y*0.79 + 1) / 2;


        _leftServo.setPosition(left);
        _rightServo.setPosition(right);

        telemetry.addData("left  ", left);
        telemetry.addData("right", right);
        /*telemetry.addData("left rear  ", _motorLeftRear.getCurrentPosition());
        telemetry.addData("right rear ", _motorRightRear.getCurrentPosition());*/
    }

    @Override
    public void stop() {

    }

    /*private void setEncoderMode(DcMotorController.RunMode mode) {
        _motorLeftFront.setChannelMode(mode);
        _motorRightFront.setChannelMode(mode);
        _motorLeftRear.setChannelMode(mode);
        _motorRightRear.setChannelMode(mode);
    }*/
}
