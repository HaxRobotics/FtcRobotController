package org.firstinspires.ftc.teamcode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LEDStrip;
import org.firstinspires.ftc.teamcode.subsystems.ShippingElementDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BLUE COMP CAROUSEL")
public class BlueAuto extends OpMode {

    // declare vision variables
    int width = 352;
    int height = 288;

    // declare necessary objects
    SampleMecanumDrive drive;
    Arm arm;
    Intake intake;
    Carousel carousel;
    WebcamName webcamName;
    OpenCvCamera webcam;
    ShippingElementDetector detector = new ShippingElementDetector(width);
    Pose2d startPose = new Pose2d(-34, 62.5, Math.toRadians(270));
    LEDStrip strip;
    TrajectorySequence trajSeq;

    @Override
    public void init() {
        initRobot();
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> arm.goTo(3))
                .waitSeconds(3.5)
                .addTemporalMarker(() -> webcam.closeCameraDevice())
                .lineToSplineHeading(new Pose2d(-34, 57, Math.toRadians(90)))
                .addTemporalMarker(() -> carousel.startPower(0.5))
                .strafeTo(new Vector2d(-56, 59.5))
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(-13, 57, Math.toRadians(270)))
                .addTemporalMarker(() -> {
                            carousel.stop();
                            arm.goTo(detector::getLocationInt);
                        }
                )
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(-12, 36))
                .addTemporalMarker(() -> intake.out(1))
                .waitSeconds(0.5)
                .addTemporalMarker(intake::stop)
                .lineToConstantHeading(new Vector2d(-34, 56.5))
                .lineToSplineHeading(new Pose2d(-56, 36, 0))
                .addTemporalMarker(() -> {
                    arm.goTo(0);
                    strip.allianceSolid();
                })
                .build();
    }

    @Override
    public void init_loop() {
        strip.detected(detector.getLocation());
        telemetry.addData("Location", detector.getLocation().name());
    }

    @Override
    public void start() {
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    @Override
    public void loop() {
        strip.detected(detector.getLocation());
        telemetry.addData("Location", detector.getLocation().name());
        drive.update();
        arm.update();
    }

    public void initRobot() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        arm = new Arm(hardwareMap, "arm");
        // add encoder levels to arm
        arm.addLevel(0).addLevel(410).addLevel(940).addLevel(1530);
        intake = new Intake(hardwareMap, "left intake", "right intake");
        carousel = new Carousel(hardwareMap, "carousel");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
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

        strip = new LEDStrip(hardwareMap, "blinkin", LEDStrip.Alliance.BLUE);
    }
}
