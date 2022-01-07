package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriverTrials extends OpMode {
    DriveTrain drive;
    Arm arm;
    Intake intake;
    Carousel carousel;
    @Override
    public void init() {
        arm = new Arm(hardwareMap, "arm");
        // add encoder levels to arm
        arm.addLevel(0).addLevel(410).addLevel(940).addLevel(1530);

        intake = new Intake(hardwareMap, "left intake", "right intake");

        drive = new DriveTrain(hardwareMap,
                "front left drive",
                "front right drive",
                "back left drive",
                "back right drive"
        );
        carousel = new Carousel(hardwareMap, "carousel");
    }

    @Override
    public void loop() {
        drive.teleDrive(
                -gamepad1.left_stick_y * 0.8,
                gamepad1.left_stick_x * 0.8,
                gamepad1.right_stick_x * 0.8);

        if (gamepad1.a) {
            arm.goTo(0);
        } else if (gamepad1.b) {
            arm.goTo(1);
        } else if (gamepad1.x) {
            arm.goTo(2);
        } else if (gamepad1.y) {
            arm.goTo(3);
        }
        arm.update();

        if (gamepad1.right_bumper) {
            intake.out();
        } else if (gamepad1.left_bumper) {
            intake.in();
        } else {
            intake.stop();
        }

        // carousel
        if (gamepad1.right_trigger > 0) {
            carousel.start();
        } else {
            carousel.stop();
        }
    }
}
