package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.objectClasses.Arm;
import org.firstinspires.ftc.teamcode.objectClasses.Carousel;
import org.firstinspires.ftc.teamcode.objectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.objectClasses.Intake;

@TeleOp
public class FirstFullTeleOp extends OpMode {
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
        telemetry.addData("Encoder Pos", arm.armMotor.getCurrentPosition());
        // drive
        drive.teleDrive(
                -gamepad1.left_stick_y * 0.8,
                gamepad1.left_stick_x * 0.8,
                gamepad1.right_stick_x * 0.8);

        if (gamepad2.a) {
            arm.goTo(0);
        } else if (gamepad2.b) {
            arm.goTo(1);
        } else if (gamepad2.x) {
            arm.goTo(2);
        } else if (gamepad2.y) {
            arm.goTo(3);
        } else {
            arm.setPower(-gamepad2.left_stick_y);
        }
        arm.update();

        // intake
        if (gamepad2.right_bumper) {
            intake.out(1);
        } else if (gamepad2.left_bumper) {
            intake.in(1);
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
