package com.qualcomm.ftcrobotcontroller.opmodes;

public class SuperK9Test extends SuperK9Base {

    /*
     * Autonomous program
     *
     * Move forward 2 feet
     */
    @Override
    public void k9Start() {
        //this.setManServoPosition(ManServoPosition.HOME);
        //this.runWithoutEncoders();
        this.runWithEncoders();
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void k9Loop() {
        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
		/* float throttle = -gamepad1.left_stick_y;
		float direction = gamepad1.left_stick_x;
		float right = throttle - direction;
		float left = throttle + direction; */
        float left = -gamepad1.left_stick_y;
        //float right = -gamepad1.right_stick_y;
        this.setPowerScaled(left, -left);

        telemetry.addData("lightOuter", this.getLightOuter());
        telemetry.addData("lightInner", this.getLightInner());
    }
}