package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Auto extends LinearOpMode {
    DriveTrain drive;
    Arm arm;
    Intake intake;
    Carousel carousel;
    ElapsedTime time = new ElapsedTime();
    int width = 352;
    int height = 288;
    int location = 0;
    // store as variable here so we can access the location
    ShippingElementDetector detector = new ShippingElementDetector(width);

    WebcamName webcamName;
    OpenCvCamera webcam;

    public void runOpMode() throws InterruptedException {
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
        carousel = new Carousel(hardwareMap, "carousel");int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(detector);

        waitForStart();

        time.reset();
        while(time.seconds() < 2 && opModeIsActive()){
            telemetry.addData("position", detector.getLocation());
            if(detector.location == ShippingElementDetector.ElementLocation.LEFT){
                location = 1;
            } else if (detector.location == ShippingElementDetector.ElementLocation.RIGHT) {
                location = 3;
            } else {
                location = 2;
            }
            telemetry.addData("position variable", location);
            telemetry.update();
        }

        // drives forward to turning position
        time.reset();
        while (time.seconds() < .3 && opModeIsActive()) {
            drive.drive(.6, 0, 0);
        }
        // turns 180 degrees so robot faces towards wall
        time.reset();
        while (time.seconds() < 1.55 && opModeIsActive()) {
            drive.drive(0, 0, .4);
        }
        // strafes so robot is pointing at carousel; just off the wall
        time.reset();
        while (time.seconds() < .7 && opModeIsActive()) {
            drive.drive(0, -.6, 0);
        }
        // drives too carousel with lower speed
        time.reset();
        while (time.seconds() < 1.5 && opModeIsActive()) {
            drive.drive(.2, 0, 0);
        }
        // spins wheel to spin one duck off carousel
        time.reset();
        while (time.seconds() < 2.5 && opModeIsActive()) {
            carousel.start();
            drive.drive(0, 0, 0);
        }
        carousel.stop();

        // backs up from carousel
        time.reset();
        while (time.seconds() < .25 && opModeIsActive()) {
            drive.drive(-.6,0,0);
        }

        // turns 180 degrees to face opposite wall/storage hub
        time.reset();
        while (time.seconds() < 1.45 && opModeIsActive()) {
            drive.drive(0,0,.4);
        }

        // strafes to line up with storage hub
        time.reset();
        while (time.seconds() < 1.5 && opModeIsActive()) {
            drive.drive(0,-.6,0);
        }

        time.reset();
        while (time.seconds() < .5 && opModeIsActive()) {
            drive.drive(0,0,0);
        }

        // drives up to storage hub
        time.reset();
        while (time.seconds() < .1 && opModeIsActive()) {
            drive.drive(.6,0,0);
        }

        time.reset();
        while (time.seconds() < 2 && opModeIsActive()) {
            drive.drive(0,0,0);
        }

        // drives up to storage hub
        /*time.reset();
        while (time.seconds() < .2 && opModeIsActive()) {
            drive.drive(.6,0,0);
        }

        time.reset();
        while (time.seconds() < .5 && opModeIsActive()) {
            drive.drive(0,0,0);
        } */

        // lifts arm to correct level
        time.reset();
        while (time.seconds() < 1 && opModeIsActive()) {
            if(location == 1) {
                arm.goTo(1);
            } else if (location == 3) {
                arm.goTo(3);
            } else {
                arm.goTo(2);
            }
        }

        time.reset();
        while (time.seconds() < 1 && opModeIsActive()) {
            intake.out(1);
        }
        intake.stop();

        time.reset();
        while (time.seconds() < 1 && opModeIsActive()) {
            drive.drive(0,0,0);
        }

        time.reset();
        while(time.seconds() < .2 && opModeIsActive()) {
            drive.drive(-.6,0,0);
        }

        time.reset();
        while (time.seconds() < 1 && opModeIsActive()) {
            drive.drive(0,.6,0);
        }
    }
}
