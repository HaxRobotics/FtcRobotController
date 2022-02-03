package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name = "2 Block Warehouse")
public class TwoBlockAuto extends OpMode {

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
                //drive to the shipping hub
                .addTemporalMarker(() -> webcam.stopStreaming())
                .lineToLinearHeading(new Pose2d(12, 24, Math.toRadians(180)))
                //move arm to correct level
                .addTemporalMarker(() -> arm.goTo(detector::getLocationInt))
                .waitSeconds(1)
                //position closer to the shipping hub
                .lineTo(new Vector2d(1.74, 24))
                //release the preloaded block
                .addTemporalMarker(() -> intake.out(1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> intake.stop())
                //go back to start position
                .lineToLinearHeading(new Pose2d(12, 63, Math.toRadians(270)))
                //strafe into warehouse
                .setVelConstraint(new MecanumVelocityConstraint(30, 15))
                .setAccelConstraint(new ProfileAccelerationConstraint(12))
                .strafeTo(new Vector2d(41, 63))
                .resetConstraints()
                //lower arm to lowest level
                .addTemporalMarker(() -> arm.goTo(0))
                .waitSeconds(1.25)
                //turn on the intake to pick up a block
                .addTemporalMarker(() -> intake.in(1))
                //drive closer to the pile of blocks
                .lineToLinearHeading(new Pose2d(51, 57.5, Math.toRadians(0)))
                .waitSeconds(0.75)
                .addTemporalMarker(() -> intake.stop())
                //raise the arm to cross the barrier
                .addTemporalMarker(() -> arm.goTo(1))
                //drive back to start position
                .lineToLinearHeading(new Pose2d(41, 63, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, 63))
                //drive to shipping hub
                .lineToLinearHeading(new Pose2d(12, 24, Math.toRadians(180)))
                //move arm to correct level
                .addTemporalMarker(() -> arm.goTo(detector::getLocationInt))
                .waitSeconds(1)
                //position closer to the shipping hub
                .lineTo(new Vector2d(1.74, 24))
                //release block
                .addTemporalMarker(() -> intake.out(1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> intake.stop())
                //return to start position
                .lineToSplineHeading(new Pose2d(12, 63, Math.toRadians(270)))
                //drive into warehouse
                .setVelConstraint(new MecanumVelocityConstraint(30, 15))
                .strafeTo(new Vector2d(41, 63))
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
        drive.update();
        arm.update();
    }
}
