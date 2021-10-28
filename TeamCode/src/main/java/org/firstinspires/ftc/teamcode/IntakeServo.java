package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class IntakeServo extends OpMode {

    CRServo servo;

    @Override
    public void init() {
        servo = hardwareMap.crservo.get("intakeServo");
    }

    @Override
    public void loop() {
        intake();
    }

    public void intake() {
        if (gamepad1.b) {
            servo.setPower(1);
        } else if (gamepad1.x) {
            servo.setPower(-1);
        } else {
            servo.setPower(0);
        }
    }
}
