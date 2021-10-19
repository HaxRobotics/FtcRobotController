package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class HolonomicDrive extends OpMode {
    // threshold used for deadzones on joysticks and making the triggers into buttons
    final double THRESHOLD = .12;

    // declare drive motors
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // multipliers for drive method
    final double strafe_speed = 1;
    final double forward_speed = 1;
    final double turn_speed = 1;

    @Override
    public void init() {
        // initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
    }

    @Override
    public void loop() {
        // call drive
        drive();
    }

    // drive according to controller inputs from driver's sticks
    public void drive() {
        // set the driver's sticks to correspond with the drive method
        // left stick y = forward/backward; left stick x = strafing; right stick x = turning
        double forwardPower = Math.abs(gamepad1.left_stick_y) < THRESHOLD ? 0 : gamepad1.left_stick_y;
        double strafePower = Math.abs(gamepad1.left_stick_x) < THRESHOLD ? 0 : gamepad1.left_stick_x;
        double turnPower = Math.abs(gamepad1.right_stick_x) < THRESHOLD ? 0 : gamepad1.right_stick_x;

        // set motor powers to correct values
        frontLeftDrive.setPower(-(forward_speed * forwardPower) + (strafe_speed * strafePower) + (turn_speed * turnPower));
        frontRightDrive.setPower(-(forward_speed * forwardPower) - (strafe_speed * strafePower) - (turn_speed * turnPower));
        backLeftDrive.setPower(-(forward_speed * forwardPower) - (strafe_speed * strafePower) + (turn_speed * turnPower));
        backRightDrive.setPower(-(forward_speed * forwardPower) + (strafe_speed * strafePower) - (turn_speed * turnPower));
    }
}
