package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TwoWheelDrive extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor intakeMotor;
    Servo servo;

    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo = hardwareMap.get(Servo.class, "servo");
    }

    public void loop () {
        double linearPower;
        double turnPower;

        linearPower = -gamepad1.left_stick_y;
        turnPower = -gamepad1.right_stick_x;

        leftMotor.setPower((linearPower) + (turnPower));
        rightMotor.setPower((linearPower) - (turnPower));

        if (gamepad1.right_bumper) {
            intakeMotor.setPower(1);
        } else if (gamepad1.left_bumper) {
            intakeMotor.setPower(-.6);
        } else {
            intakeMotor.setPower(0);
        }
        if (gamepad1.a) {
            servo.setPosition(1);
        } else if (gamepad1.b) {
            servo.setPosition(-1);
        }
    }


}
