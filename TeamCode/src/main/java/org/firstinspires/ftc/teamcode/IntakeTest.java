package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntakeTest extends OpMode {
    Intake intake;
    @Override
    public void init() {
        intake = new Intake(hardwareMap, "leftIntake", "rightIntake");
    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            if (intake.isRunning()) {
                intake.stop();
            } else {
                intake.out(1);
            }
        }
        if (gamepad2.left_bumper) {
            if (intake.isRunning()) {
                intake.stop();
            } else {
                intake.in(1);
            }
        }
    }
}
