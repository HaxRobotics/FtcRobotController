package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class DriveTrain {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    public DriveTrain(@NonNull HardwareMap hw, String frontLeftName, String frontRightName, String backLeftName,
            String backRightName) {
        // get motors
        frontLeft = hw.get(DcMotor.class, frontLeftName);
        frontRight = hw.get(DcMotor.class, frontRightName);
        backLeft = hw.get(DcMotor.class, backLeftName);
        backRight = hw.get(DcMotor.class, backRightName);
        // necessary for mecanum drive
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DriveTrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    private void normalize(double[] wheelSpeeds) {
        // find max magnitude power
        double maxMagnitude = Arrays.stream(wheelSpeeds).map(Math::abs).max().orElse(1);
        // scale down if needed
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }

    public void teleDrive(double forward, double strafe, double rotate) {
        drive(squareInput(forward), squareInput(strafe), squareInput(rotate));
    }

    public void drive(double forward, double strafe, double rotate) {
        double[] wheelSpeeds = {
                // front left
                forward + strafe + rotate,
                // front right
                forward - strafe - rotate,
                // back left
                forward - strafe + rotate,
                // back right
                forward + strafe - rotate
        };

        // normalize powers
        normalize(wheelSpeeds);

        // set power
        frontLeft.setPower(wheelSpeeds[0]);
        frontRight.setPower(wheelSpeeds[1]);
        backLeft.setPower(wheelSpeeds[2]);
        backRight.setPower(wheelSpeeds[3]);
    }

    /**
     * Square magnitude of number while keeping the sign.
     */
    protected double squareInput(double input) {
        return input * Math.abs(input);
    }
}
