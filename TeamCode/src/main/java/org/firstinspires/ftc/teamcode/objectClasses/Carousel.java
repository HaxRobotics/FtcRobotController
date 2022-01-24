package org.firstinspires.ftc.teamcode.objectClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
// class for motor controlling carousel
public class Carousel {
    private final DcMotorEx carouselMotor;
    private double toggleSpeed;

    public Carousel(HardwareMap hw, String motorName) {
        carouselMotor = hw.get(DcMotorEx.class, motorName);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toggleSpeed = carouselMotor.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    public void start() {
        carouselMotor.setVelocity(toggleSpeed);
    }
    public void redStart() {
        carouselMotor.setVelocity(-toggleSpeed);
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
    public void setToggleSpeed(double newSpeed) {
        toggleSpeed = newSpeed;
    }

    public double getToggleSpeed() {
        return toggleSpeed;
    }
}
