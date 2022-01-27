package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.objectClasses.Arm;
import org.firstinspires.ftc.teamcode.objectClasses.Carousel;
import org.firstinspires.ftc.teamcode.objectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.objectClasses.Intake;
import org.firstinspires.ftc.teamcode.objectClasses.LEDStrip;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "COMP TELEOP")
public class FirstFullTeleOp extends OpMode {
    int width = 352;
    int height = 288;

    StandardTrackingWheelLocalizer localizer;
    DriveTrain drive;
    Arm arm;
    Intake intake;
    Carousel carousel;
    LEDStrip strip;
    boolean first = true;
    FtcDashboard dashboard;
    CRServo hook;
    ShippingElementDetector detector;
    OpenCvCamera webcam;
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

        strip = new LEDStrip(hardwareMap, "blinkin", LEDStrip.Alliance.RED);
        strip.allianceSolid();
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        localizer.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(270)));
        detector = new ShippingElementDetector(width);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // empty
            }
        });

        // set opencv pipeline
        webcam.setPipeline(detector);

        dashboard = FtcDashboard.getInstance();
        hook = hardwareMap.get(CRServo.class, "hook");
    }

    @Override
    public void loop() {
        telemetry.addData("Location", detector.getLocation().name());
        if (gamepad1.left_stick_button) {
            strip.alliance = LEDStrip.Alliance.BLUE;
        } else if (gamepad1.right_stick_button) {
            strip.alliance = LEDStrip.Alliance.RED;
        } else if (gamepad1.dpad_up) {
            strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        localizer.update();

        Pose2d pose = localizer.getPoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        DashboardUtil.drawRobot(canvas, pose);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading", pose.getHeading());
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
        if (gamepad2.right_trigger > 0) {
            carousel.start();
        } else if (gamepad2.left_trigger > 0) {
            carousel.redStart();
        } else if (gamepad1.right_trigger > 0) {
            carousel.start();
        } else if (gamepad1.left_trigger > 0) {
            carousel.redStart();
        } else {
            carousel.stop();
        }



        // Shipping element
        if (gamepad2.dpad_up) {
            hook.setPower(-1);
        } else if (gamepad2.dpad_down) {
            hook.setPower(1);
        } else {
            hook.setPower(0);
        }
        if (gamepad2.right_stick_button) {
            arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (getRuntime() > 90 && first) {
            strip.allianceBlink();
            first = false;
        }
    }
}
