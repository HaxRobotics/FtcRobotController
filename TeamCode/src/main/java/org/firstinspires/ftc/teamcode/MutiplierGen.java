package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;

@Autonomous
public class MutiplierGen extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    // fl, fr, bl, br
    double[] velocities = new double[4];

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        frontRight = hardwareMap.get(DcMotor.class, "front right");
        backLeft = hardwareMap.get(DcMotor.class, "back left");
        backRight = hardwareMap.get(DcMotor.class, "back right");

        waitForStart();
        // run at half power to make encoders more accurate
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        // let motors get to full speed
        sleep(4000);
        // get encoder ticks per second
        velocities[0] = ((DcMotorEx) frontLeft).getVelocity();
        velocities[1] = ((DcMotorEx) frontRight).getVelocity();
        velocities[2] = ((DcMotorEx) backLeft).getVelocity();
        velocities[3] = ((DcMotorEx) backRight).getVelocity();

        // match to slowest wheel
        double minVel = Arrays.stream(velocities).min().orElse(0);


        double[] multipliers = Arrays.stream(velocities).map(vel -> minVel / vel).toArray();

        telemetry.addData("Multipliers", Arrays.toString(multipliers));
    }
}
