package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LinearSlide extends OpMode {
    //declares linear slide motor
    DcMotor linearSlide;
    //declares motor powers
    final double EXTEND_POWER = .5;
    final double RETRACT_POWER = -.5;

    public void init(){
        //initializes linear slide motor
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
    }

    public void loop(){
        //calls method
        linearSlide();
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

}
