package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.objectClasses.Arm;
import org.firstinspires.ftc.teamcode.objectClasses.Carousel;
import org.firstinspires.ftc.teamcode.objectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.objectClasses.Intake;
import org.firstinspires.ftc.teamcode.objectClasses.LEDStrip;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "BLUE AUTO WAREHOUSE")
public class WarehouseAuto extends OpMode {

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
    Pose2d startPose = new Pose2d(12, 62, Math.toRadians(270));
    LEDStrip strip;
    TrajectorySequence trajSeq;

    public void initRobot() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        arm = new Arm(hardwareMap, "arm");
        // add encoder levels to arm
        arm.addLevel(0).addLevel(410).addLevel(940).addLevel(1530);
        intake = new Intake(hardwareMap, "left intake", "right intake");
        carousel = new Carousel(hardwareMap, "carousel");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

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

    @Override
    public void init() {
        initRobot();
        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, 24, Math.toRadians(180)))
                .addTemporalMarker(() -> arm.goTo(detector::getLocationInt))
                .waitSeconds(1)
                .lineTo(new Vector2d(1.74, 24))
                .addTemporalMarker(() -> intake.out(1))
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(12, 59))
                .lineToSplineHeading(new Pose2d(12, 62, Math.toRadians(270)))
                .strafeTo(new Vector2d(41, 61))
                .lineToSplineHeading(new Pose2d(41, 54, 0))
                .addTemporalMarker(() -> arm.goTo(0))

                .build();

    }
    @Override
    public void init_loop() {
        strip.detected(detector.getLocation());
        telemetry.addData("Location", detector.getLocation().name());
    }
    @Override
    public void start() {
        arm.goTo(1);
        drive.followTrajectorySequenceAsync(trajSeq);
    }
    @Override
    public void loop() {
        drive.update();
        arm.update();
    }
}
