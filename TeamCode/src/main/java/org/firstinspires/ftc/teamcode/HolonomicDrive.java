package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class HolonomicDrive {
    //array for motors
    private final DcMotor[] motors;
    private final double[] multiplier;


    //puts motor into the array
    public HolonomicDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        multiplier = new double[]{1, 1, 1, 1};
    }

    public HolonomicDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, double frontLeftMultiplier, double frontRightmultiplier, double backLeftMultiplyer, double backRightMultiplier) {
        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        multiplier = new double[]{frontLeftMultiplier, frontRightmultiplier, backLeftMultiplyer, backRightMultiplier};
    }

    // drive according to controller inputs from driver's sticks
    public void drive(double forwardPower, double strafePower, double turnPower) {
        double[] wheelSpeeds = new double[4];
        //sets motor values
        wheelSpeeds[WheelPositions.FRONT_LEFT.value] = (-forwardPower) + (strafePower) + (turnPower);
        wheelSpeeds[WheelPositions.FRONT_RIGHT.value] = (-forwardPower) - (strafePower) - (turnPower);
        wheelSpeeds[WheelPositions.BACK_LEFT.value] = (-forwardPower) - (strafePower) + (turnPower);
        wheelSpeeds[WheelPositions.BACK_RIGHT.value] = (-forwardPower) + (strafePower) - (turnPower);
        //sets motor power to values
        setPower(wheelSpeeds);
    }

    //sets motor powers
    public void setPower(double... powers) {
        //checks if array lengths are the same
        if (powers.length != motors.length) {
            throw new RuntimeException("array discrepancy when setting motor powers");
        }
        //sets motor power
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i] * multiplier[i]);
        }
    }

}
