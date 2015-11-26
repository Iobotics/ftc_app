package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Student on 11/25/2015.
 */
public class TankbotReadOnly extends TankbotBase {

    DcMotorController _wheelControllerFront;
    DcMotorController _wheelControllerRear;

    public void TBInit() {
        _wheelControllerFront = hardwareMap.dcMotorController.get("frontMotors");
        _wheelControllerFront.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);

        _wheelControllerRear = hardwareMap.dcMotorController.get("rearMotors");
        _wheelControllerRear.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
    }

    public void TBStart() {
        super.resetEncoders();
    }

    public void TBLoop() {
        telemetry.addData("Left encoder", String.format("%.2f", super.getLeftPositionInches()));
        telemetry.addData("Right encoder", String.format("%.2f", super.getRightPositionInches()));
    }
}
