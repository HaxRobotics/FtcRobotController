package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class HolonomicDrive {
    // multipliers for drive method
    final double strafe_speed = 1;
    final double forward_speed = 1;
    final double turn_speed = 1;

    private final DcMotor[] motors;

    public HolonomicDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, DcMotor[] motors) {
        this(frontLeft, frontRight, backLeft, backRight);

    }

    public HolonomicDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        motors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight};
    }

    // drive according to controller inputs from driver's sticks
    public void drive(double forwardPower, double strafePower, double turnPower) {
        double[] wheelSpeeds = new double[4];

        wheelSpeeds[WheelPositions.FRONT_LEFT.value] = (-forwardPower * forward_speed) + (strafePower * strafe_speed) + (turnPower * turn_speed);
        wheelSpeeds[WheelPositions.FRONT_RIGHT.value] = (-forwardPower * forward_speed) - (strafePower * strafe_speed) - (turnPower * turn_speed);
        wheelSpeeds[WheelPositions.BACK_LEFT.value] = (-forwardPower * forward_speed) - (strafePower * strafe_speed) + (turnPower * turn_speed);
        wheelSpeeds[WheelPositions.BACK_RIGHT.value] = (-forwardPower * forward_speed) + (strafePower * strafe_speed) - (turnPower * turn_speed);

        setPower(wheelSpeeds);
    }

    public void setPower(double... powers) {
        if(powers.length != motors.length) {
            throw new RuntimeException("array discrepancy when setting motor powers");
        }

        for(int i = 0; i < motors.length; i++) {
            motors[i].setPower(powers[i]);
        }
    }

}
