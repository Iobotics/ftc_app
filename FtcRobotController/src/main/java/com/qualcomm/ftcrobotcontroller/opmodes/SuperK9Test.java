package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class SuperK9Test extends OpMode {

    Servo _servo1, _servo2, _servo3, _servo4;

    @Override
    public void init() {

        _servo1 = hardwareMap.servo.get("buttonServo");
        _servo1.setPosition(0.5);
        _servo2 = hardwareMap.servo.get("manServo");
        _servo2.setDirection(Servo.Direction.REVERSE);
        _servo2.setPosition(0.5);

        _servo3 = hardwareMap.servo.get("plowMotor");
        _servo3.setPosition(0.5);
        _servo4 = hardwareMap.servo.get("dozerMotor");
        _servo4.setPosition(0.5);
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
		/* float throttle = -gamepad1.left_stick_y;
		float direction = gamepad1.left_stick_x;
		float right = throttle - direction;
		float left = throttle + direction; */
        float left = (1 - gamepad1.left_stick_y)/2;
        //float right = -gamepad1.right_stick_y;
        _servo1.setPosition(left);
        _servo2.setPosition(left);

        telemetry.addData("power", left);
    }
}