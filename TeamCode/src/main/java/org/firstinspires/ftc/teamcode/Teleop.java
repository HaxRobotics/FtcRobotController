package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// teleop class for robot with the linear slide
@TeleOp
public class Teleop extends OpMode {
    // threshold used for deadzones on joysticks and making the triggers into buttons
    final double THRESHOLD = .12;

    // declare drive constructor
    HolonomicDrive drive;

    // declare drive motors
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // declare drive multipliers
    double frontLeftPower = .2;
    double frontRightPower = .2;
    double backLeftPower = .2;
    double backRightPower = .2;

    // declare linear slide motor
    DcMotor linearSlide;
    // declare intake servo
    CRServo intakeServo;
    // declare outtake Servo
    Servo outtakeServo;

    // declare booleans for outtake toggle
    public boolean isA = false;
    public boolean wasA = false;
    public boolean servoOn = false;

    //declares motor powers
    final double EXTEND_POWER = .5;
    final double RETRACT_POWER = -.5;

    public void init(){
        // initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // initialize drive obj
        drive = new HolonomicDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        // initialize linear slide motor
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        // initialize intake CRServo
        intakeServo = hardwareMap.crservo.get("intakeServo");
        // initialize outtake servo
        outtakeServo = hardwareMap.get(Servo.class, "outtakeServo");
    }

    public void loop(){
        drive();
        linearSlide();
        intake();
        outtake();
    }

    public void drive() {
        // set the driver's sticks to correspond with the drive method
        // left stick y = forward/backward; left stick x = strafing; right stick x = turning
        double forwardPower = Math.abs(gamepad1.left_stick_y) < THRESHOLD ? 0 : gamepad1.left_stick_y;
        double strafePower = Math.abs(gamepad1.left_stick_x) < THRESHOLD ? 0 : gamepad1.left_stick_x;
        double turnPower = Math.abs(gamepad1.right_stick_x) < THRESHOLD ? 0 : gamepad1.right_stick_x;

        drive.drive(forwardPower, strafePower, turnPower);
    }

    public void linearSlide() {
        //detects bumpers and sets power
        if (gamepad1.right_bumper) {
            linearSlide.setPower(EXTEND_POWER);
        } else if (gamepad1.left_bumper) {
            linearSlide.setPower(RETRACT_POWER);
        } else {
            linearSlide.setPower(0);
        }
    }

    public void intake() {
        if (gamepad1.b) {
            intakeServo.setPower(1);
        } else if (gamepad1.x) {
            intakeServo.setPower(-1);
        } else {
            intakeServo.setPower(0);
        }
    }

    public void outtake() {
        // track history of button
        if((isA = gamepad1.a) && !wasA) {
            if(servoOn) {
                // if the servo is on and a is pressed, turn shooter off
                outtakeServo.setPosition(.5);
            } else {
                // if the servo is off and a is pressed, turn shooter on
                outtakeServo.setPosition(1);
            }
            servoOn = !servoOn;
        }
        wasA = isA;
    }
}
