package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class MaxVelocity extends OpMode {
    DcMotorEx arm;
    double currVelocity;
    double maxVelocity = 0;
    @Override
    public void init() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            arm.setPower(1);
        } else if (gamepad1.b) {
            arm.setPower(0);
        }
        currVelocity = arm.getVelocity();
        if (currVelocity > maxVelocity) {
            maxVelocity = currVelocity;
        }
        telemetry.addData("Current Velocity", currVelocity);
        telemetry.addData("Max Velocity", maxVelocity);

    }
}
