package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LinearSlideElevator extends OpMode {

    Servo linearSlideElevator;

    public boolean isY = false;
    public boolean wasY = false;
    public boolean elevatorOn = false;

    @Override
    public void init() {
        linearSlideElevator = hardwareMap.get(Servo.class, "linearSlideElevator");
    }

    @Override
    public void loop() {
        linearSlideElevator();
    }

    public void linearSlideElevator() {
        // track history of button
        if((isY = gamepad1.y) && !wasY) {
            if(elevatorOn) {
                // if the servo is on and y is pressed, turn elevator off
                linearSlideElevator.setPosition(.75);
            } else {
                // if the servo is off and y is pressed, turn elevator on
                linearSlideElevator.setPosition(0);
            }
            elevatorOn = !elevatorOn;
        }
        wasY = isY;
    }
}


