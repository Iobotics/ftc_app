package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Student on 9/9/2015.
 */
public class OldBotArcadeTeleOP extends OpMode {
    // Declare motors //
    DcMotor motorFrontLeft;
    DcMotor motorMidLeft;
    DcMotor motorRearLeft;
    DcMotor motorFrontRight;
    DcMotor motorMidRight;
    DcMotor motorRearRight;
    DcMotor motorIntake;
    DcMotor motorLift;

    private static final int gamepadInvert = -1; //Gamepad returns -1 as full forward; we want it to be 1

    public OldBotArcadeTeleOP() {

    }

    @Override
    public void init() {
        // Assign motors //
        motorFrontLeft = hardwareMap.dcMotor.get("l1");
        motorMidLeft = hardwareMap.dcMotor.get("l2");
        motorRearLeft = hardwareMap.dcMotor.get("l3");
        motorFrontRight = hardwareMap.dcMotor.get("r1");
        motorMidRight = hardwareMap.dcMotor.get("r2");
        motorRearRight = hardwareMap.dcMotor.get("r3");
        motorIntake = hardwareMap.dcMotor.get("intake");
        motorLift = hardwareMap.dcMotor.get("lift");

        // Reverse left motors and lift //
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorMidLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // Get controller inputs //
        float leftStickX = gamepad1.left_stick_x;
        float rightStickY = gamepad1.right_stick_y * gamepadInvert;
        setArcade(leftStickX,  rightStickY);


        /*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", Range.clip(rightStickY - leftStickX, -1, 1)));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", Range.clip(rightStickY + leftStickX, -1, 1)));
    }

    public void stop() {


    }

    public void setArcade(float leftIn, float rightIn) {
        // Set left / right powers... this is the algorithm stuff //
        float right = rightIn - leftIn;
        float left  = rightIn + leftIn;

        // Clip input //
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        setTank(left, right);
    }

    public void setTank(float left, float right) {
        motorFrontLeft.setPower(left);
        motorMidLeft.setPower(left);
        motorRearLeft.setPower(left);
        motorFrontRight.setPower(right);
        motorMidRight.setPower(right);
        motorRearRight.setPower(right);
    }



}
