package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
@Config
@Disabled
public class CarouselTest extends OpMode {
    DcMotorEx carousel;
    public static double actual = 0;
    boolean velocity = true;
    public static double power = 0;
    @Override
    public void init() {
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (velocity) {
            carousel.setVelocity(actual);
        } else {
            carousel.setPower(power);
        }
        if (gamepad1.a) {
            velocity = true;
        }
        if (gamepad1.b) {
            velocity = false;
        }
        telemetry.addData("Power", carousel.getPower());
        telemetry.addData("Speed (ticks)", carousel.getVelocity());
        telemetry.addData("Speed (rev)", (carousel.getVelocity() / 537.6) * 60);
    }
}
