package org.firstinspires.ftc.teamcode.linearSlideRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HolonomicDrive;

// teleop class for robot with the linear slide
@TeleOp
public class Teleop extends OpMode {
    // threshold used for deadzones on joysticks and making the triggers into buttons
    final double THRESHOLD = .12;

    // declare drive constructor
    HolonomicDrive drive;

    // declare drive motors
    /*DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // declare drive multipliers
    double frontLeftPower = .2;
    double frontRightPower = .2;
    double backLeftPower = .2;
    double backRightPower = .2; */

    // declare linear slide motor
    DcMotor linearSlide;
    // declare linear slide elevator servo
    Servo linearSlideElevator;
    // declare intake servo
    CRServo intakeServo;

    // declare booleans for elevator toggle
    public boolean isY = false;
    public boolean wasY = false;
    public boolean elevatorOn = false;

    // declare linear slide motor powers
    final double EXTEND_POWER = .5;
    final double RETRACT_POWER = -.5;

    // initializes all hardware components
    public void init(){
        // initialize drive motors
       /* frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive"); */

        // initialize drive obj
       // drive = new HolonomicDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        // initialize linear slide motor
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        // initialize linear slide elevator servo
        linearSlideElevator = hardwareMap.get(Servo.class, "linearSlideElevator");
        // initialize intake CRServo
        intakeServo = hardwareMap.crservo.get("intakeServo");
    }

    // calls all methods
    public void loop(){
       // drive();
        linearSlide();
        linearSlideElevator();
        intake();
    }

    // control drive motors
    /*public void drive() {
        // set the driver's sticks to correspond with the drive method
        // left stick y = forward/backward; left stick x = strafing; right stick x = turning
        double forwardPower = Math.abs(gamepad1.left_stick_y) < THRESHOLD ? 0 : gamepad1.left_stick_y;
        double strafePower = Math.abs(gamepad1.left_stick_x) < THRESHOLD ? 0 : gamepad1.left_stick_x;
        double turnPower = Math.abs(gamepad1.right_stick_x) < THRESHOLD ? 0 : gamepad1.right_stick_x;

        drive.drive(forwardPower, strafePower, turnPower);
    } */

    // control linear slide motor
    public void linearSlide() {
        // checks if driver's bumpers are pressed and sets power accordingly
        if (gamepad1.right_bumper) {
            linearSlide.setPower(EXTEND_POWER);
        } else if (gamepad1.left_bumper) {
            linearSlide.setPower(RETRACT_POWER);
        } else {
            linearSlide.setPower(0);
        }
    }

    // controls linear slide's elevation
    public void linearSlideElevator() {
        // track history of driver's y button
        if((isY = gamepad1.y) && !wasY) {
            if(elevatorOn) {
                // if the servo is on and y is pressed, turn elevator off
                linearSlideElevator.setPosition(.25);
            } else {
                // if the servo is off and y is pressed, turn elevator on
                linearSlideElevator.setPosition(1);
            }
            elevatorOn = !elevatorOn;
        }
        wasY = isY;
    }

    // controls intake system
    public void intake() {
        // checks if driver's b/x buttons are pressed and sets crservo power accordingly
        if (gamepad1.b) {
            intakeServo.setPower(1);
        } else if (gamepad1.x) {
            intakeServo.setPower(-1);
        } else {
            intakeServo.setPower(0);
        }
    }
}