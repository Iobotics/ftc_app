package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Student on 10/30/2015.
 */
public class TankbotTest2 extends OpMode {

    DcMotor _motorRightFront;
    DcMotor _motorRightRear;
    DcMotor _motorLeftFront;
    DcMotor _motorLeftRear;

    DcMotorController _wheelControllerFront;

    Servo _manServo;

    @Override
    public void init() {
        _motorRightFront = hardwareMap.dcMotor.get("rightFront");
        _motorRightRear  = hardwareMap.dcMotor.get("rightRear");
        _motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        _motorLeftRear = hardwareMap.dcMotor.get("leftRear");

        _manServo = hardwareMap.servo.get("manServo");

        _wheelControllerFront = hardwareMap.dcMotorController.get("wheelMotorFront");
        _wheelControllerFront.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
    }

    @Override
    public void loop() {
        if(_wheelControllerFront.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {
            telemetry.addData("Left encoder", _motorLeftFront.getCurrentPosition());
            telemetry.addData("Right encoder", _motorRightFront.getCurrentPosition());
        }
        float pos = (gamepad1.left_stick_y + 1) / 2;
        _manServo.setPosition(pos);
        telemetry.addData("ServoPos", pos);
    }
}
