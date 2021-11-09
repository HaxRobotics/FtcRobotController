package org.firstinspires.ftc.teamcode.teleopCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TwoWheelDrive extends OpMode {
    DcMotor leftmotor;
    DcMotor rightmotor;

    public void init() {
        leftmotor = hardwareMap.get(DcMotor.class, "leftmotor");
        rightmotor = hardwareMap.get(DcMotor.class, "rightmotor");
    }

    public void loop () {
        double linearpower;
        double turnpower;

        linearpower = gamepad1.left_stick_y;
        turnpower = gamepad1.right_stick_x;

        leftmotor.setPower((linearpower) + (turnpower));
        rightmotor.setPower((linearpower) - (turnpower));
    }

}
