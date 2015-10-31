package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Student on 10/30/2015.
 */
public class TankbotTest extends OpMode {

    final static int    ENCODER_TICKS_PER_REV = 1120; // Neverest 40
    final static int    WHEEL_DIAMETER        = 2;    // inches / REV... we're using tank treads so this is really a gear diameter
    final static double INCHES_PER_TICK       = (WHEEL_DIAMETER * Math.PI) / ENCODER_TICKS_PER_REV;

    DcMotor _motorRightFront;
    DcMotor _motorRightRear;
    DcMotor _motorLeftFront;
    DcMotor _motorLeftRear;

    DcMotorController wheelControllerFront;
    DcMotorController wheelControllerRear;

    private enum StateModes {
        READ,
        WRITE,
        WAIT_FOR_SWITCH
    }

    StateModes _currentState;

    private double _leftPositionInches;
    private double _rightPositionInches;

    @Override
    public void init() {
        _motorRightFront = hardwareMap.dcMotor.get("rightFront");
        _motorRightRear  = hardwareMap.dcMotor.get("rightRear");
        _motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        _motorLeftRear = hardwareMap.dcMotor.get("leftRear");

        _motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        _motorLeftRear.setDirection(DcMotor.Direction.REVERSE);

        wheelControllerFront = hardwareMap.dcMotorController.get("wheelMotorFront");
        wheelControllerFront.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        wheelControllerRear = hardwareMap.dcMotorController.get("wheelMotorRear");
        wheelControllerRear.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

        _currentState = StateModes.WAIT_FOR_SWITCH;
    }

    @Override
    public void loop() {
        double left  = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        switch(_currentState) {
            case READ:
                _leftPositionInches = this.getLeftPositionInches();
                _rightPositionInches = this.getRightPositionInches();

                telemetry.addData("Left encoder", String.format("%.2f", _leftPositionInches));
                telemetry.addData("Right encoder", String.format("%.2f", _rightPositionInches));

                wheelControllerFront.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                //wheelControllerRear.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
                _currentState = StateModes.WAIT_FOR_SWITCH;
                break;
            case WRITE:
                setPowerScaled(left, right);

                wheelControllerFront.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                //wheelControllerRear.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
                _currentState = StateModes.WAIT_FOR_SWITCH;
                break;
            case WAIT_FOR_SWITCH:
                switch(wheelControllerFront.getMotorControllerDeviceMode()) {
                    case SWITCHING_TO_READ_MODE:
                    case SWITCHING_TO_WRITE_MODE:
                        break;
                    case READ_ONLY:
                        _currentState = StateModes.READ;
                        break;
                    case WRITE_ONLY:
                        _currentState = StateModes.WRITE;
                        break;
                }
        }
    }

    private void setPowerScaled(double left, double right) {
        left  = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        setPower(left, right);
    }

    private void setPower(double left, double right) {
        _motorRightFront.setPower(right);
        //_motorRightRear.setPower(right);
        _motorLeftFront.setPower(left);
        //_motorLeftRear.setPower(left);
    }

    private int getLeftEncoder() {
        int pos = _motorLeftFront.getCurrentPosition();
        return pos;
    }

    private int getRightEncoder() {
        int pos = _motorRightFront.getCurrentPosition();
        return pos;
    }

    private double getLeftPositionInches() {
        return this.getLeftEncoder() * INCHES_PER_TICK;
    }

    private double getRightPositionInches() {
        return this.getRightEncoder() * INCHES_PER_TICK;
    }
}
