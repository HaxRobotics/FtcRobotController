package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// class for motor controlling carousel
public class Carousel {
    public final DcMotorEx carouselMotor;
    private double toggleSpeed;

    public static final double ticksPerRev = 537.6;

    public Carousel(@NonNull HardwareMap hw, String motorName) {
        carouselMotor = hw.get(DcMotorEx.class, motorName);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toggleSpeed = 200;

    }

    private double rpmToTicks(double rpm) {
        return (rpm / 60) * ticksPerRev;
    }


    public void start() {
        carouselMotor.setVelocity(rpmToTicks(toggleSpeed));
    }

    public void redStart() {
        carouselMotor.setVelocity(-rpmToTicks(toggleSpeed));
    }

    public void startPower(double power) {
        carouselMotor.setPower(power);
    }

    public void redStartPower(double power) {
        startPower(-power);
    }

    public void stop() {
        carouselMotor.setVelocity(0);
    }

    public double getToggleSpeed() {
        return toggleSpeed;
    }

    public void setToggleSpeed(double newSpeed) {
        toggleSpeed = newSpeed;
    }
}
