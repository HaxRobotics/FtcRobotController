package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OuttakeServoTest extends OpMode {

    Servo servo;

    public boolean isA = false;
    public boolean wasA = false;
    public boolean servoOn = false;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        servo();
    }

    public void servo() {
        // track history of button
        if((isA = gamepad1.a) && !wasA) {
            if(servoOn) {
                // if the servo is on and a is pressed, turn shooter off
                servo.setPosition(.5);
            } else {
                // if the servo is off and a is pressed, turn shooter on
                servo.setPosition(1);
            }
            servoOn = !servoOn;
        }
        wasA = isA;
    }
}
