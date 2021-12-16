package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.objectClasses.Arm;
import org.firstinspires.ftc.teamcode.objectClasses.Carousel;
import org.firstinspires.ftc.teamcode.objectClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.objectClasses.Intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
    Blue alliance auto (run on side closest to carousel)
        detect shipping element
        carousel
        unload preloaded block on storage hub
        park in storage unit
 */
@Autonomous
public class BlueAuto extends LinearOpMode {
    // declare vision variables
    int width = 352;
    int height = 288;
    int location = 0;

    // declare necessary objects
    DriveTrain drive;
    Arm arm;
    Intake intake;
    Carousel carousel;
    WebcamName webcamName;
    OpenCvCamera webcam;
    ShippingElementDetector detector = new ShippingElementDetector(width);
    ElapsedTime time = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        // initialize necessary objects
        drive = new DriveTrain(hardwareMap,
                "front left drive",
                "front right drive",
                "back left drive",
                "back right drive"
        );
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
            }
        });

        // set opencv pipeline
        webcam.setPipeline(detector);

        // call waitForStart()
        waitForStart();

        // detect shipping element and it's location
        /*
            LEFT positioning means location equals one and the block goes on bottom plate
            RIGHT positioning means location equals three and the block goes on top plate
            MIDDLE positioning means location equals two and the block goes on middle plate
                location is set to two as a default
        */
        while (time.seconds() < 2) {
            // update location as seen by webcam
            telemetry.addData("position", detector.getLocation());
            if (detector.location == ShippingElementDetector.ElementLocation.LEFT) {
                location = 1;
            } else if (detector.location == ShippingElementDetector.ElementLocation.RIGHT) {
                location = 3;
            } else {
                location = 2;
            }
            // update what int location is equal to
            telemetry.addData("position variable", location);
            telemetry.update();
        }

        // close camera after location variable is set above
        webcam.closeCameraDevice();

        // drive forward to turning position - time dependent on battery voltage
        if(getBatteryVoltage() < 13.8) {
            time.reset();
            while (time.seconds() < .3 && opModeIsActive()) {
                drive.drive(.6, 0, 0);
            }
        } else {
            time.reset();
            while (time.seconds() < .2 && opModeIsActive()) {
                drive.drive(.6, 0, 0);
            }
        }

        // turn 180 degrees so robot faces wall
        time.reset();
        while (time.seconds() < 1.45 && opModeIsActive()) {
            drive.drive(0, 0, .4);
        }

        // strafe to carousel
        time.reset();
        while (time.seconds() < .7 && opModeIsActive()) {
            drive.drive(0, -.6, 0);
        }

        // drive to carousel at a low speed if battery voltage is less than 13.8
        if(getBatteryVoltage() < 13.8) {
            time.reset();
            while (time.seconds() < 1.5 && opModeIsActive()) {
                drive.drive(.2, 0, 0);
            }
        }

        // spin carousel wheel
        time.reset();
        while (time.seconds() < 2.5 && opModeIsActive()) {
            carousel.start();
            drive.drive(0, 0, 0);
        }
        // stop carousel wheel
        carousel.stop();

        // back up from carousel - time dependent on battery voltage
        if (getBatteryVoltage() > 13.3) {
            time.reset();
            while (time.seconds() < .1 && opModeIsActive()) {
                drive.drive(-.6, 0, 0);
            }
        } else {
            time.reset();
            while (time.seconds() < .25 && opModeIsActive()) {
                drive.drive(-.6, 0, 0);
            }
        }
        // turn 180 degrees to face storage hub - time dependent on battery voltage
        if (getBatteryVoltage() >= 14) {
            time.reset();
            while (time.seconds() < 2.3 && opModeIsActive()) {
                drive.drive(0, 0, .4);
            }
        } else if (getBatteryVoltage() >= 13.675) {
            time.reset();
            while (time.seconds() < 1.6 && opModeIsActive()) {
                drive.drive(0, 0, .4);
            }
        } else if (getBatteryVoltage() >= 13.5) {
            time.reset();
            while (time.seconds() < 1.5 && opModeIsActive()) {
                drive.drive(0, 0, .4);
            }
        } else {
            time.reset();
            while (time.seconds() < 1.4 && opModeIsActive()) {
                drive.drive(0, 0, .4);
            }
        }

        // strafe to line up with storage hub - time dependent on battery voltage
        if(getBatteryVoltage() > 13.75) {
            time.reset();
            while (time.seconds() < 1.3 && opModeIsActive()) {
                drive.drive(0,-.6,0);
            }
        } else {
            time.reset();
            while (time.seconds() < 1.5 && opModeIsActive()) {
                drive.drive(0,-.6,0);
            }
        }

        // stop robot and allow arm to rise
        time.reset();
        while (time.seconds() < .5 && opModeIsActive()) {
            drive.drive(0,0,0);
        }

        // lift arm to correct level - based on location integer
        /*
            location equals one, arm equals level one
            location equals three, arm equals level three
            location equals two, arm equals level two
                arm is set to two as a default
         */
        time.reset();
        while (time.seconds() < 2 && opModeIsActive()) {
            // update arm position
            telemetry.addData("Encoder Pos", arm.armMotor.getCurrentPosition());
            telemetry.update();
            if(location == 1) {
                arm.goTo(1);
            } else if(location == 3) {
                arm.goTo(3);
            } else {
                arm.goTo(2);
            }
            // ensure that arm stops at correct position
            if(arm.atLevel(location)) {
                arm.setPower(0);
            }
        }

        // drive closer to storage hub - time dependent on battery voltage
        if (getBatteryVoltage() > 13.4) {
            time.reset();
            while (time.seconds() < .35 && opModeIsActive()) {
                drive.drive(.6,0,0);
            }
        } else {
            time.reset();
            while (time.seconds() < .45 && opModeIsActive()) {
                drive.drive(.6,0,0);
            }
        }

        // stop robot and allow for intake to unload block
        time.reset();
        while (time.seconds() < .5 && opModeIsActive()) {
            drive.drive(0,0,0);
        }

        // unload block from robot
        time.reset();
        while (time.seconds() < 1 && opModeIsActive()) {
            intake.out(1);
        }
        // stop intake
        intake.stop();

        // stop robot and allow to reset
        time.reset();
        while (time.seconds() < 1 && opModeIsActive()) {
            drive.drive(0,0,0);
        }

        // drive away from storage hub - time dependent on battery voltage
        if(getBatteryVoltage() > 13.5) {
            time.reset();
            while(time.seconds() < .2 && opModeIsActive()) {
                drive.drive(-.6, 0, 0);
            }
        } else {
            time.reset();
            while(time.seconds() < .3 && opModeIsActive()) {
                drive.drive(-.6,0,0);
            }
        }

        // strafe toward storage unit - time dependent on battery voltage
        if(getBatteryVoltage() > 13.3) {
            time.reset();
            while (time.seconds() < 1.25 && opModeIsActive()) {
                drive.drive(0,.6,0);
            }
        } else {
            time.reset();
            while (time.seconds() < 1.55 && opModeIsActive()) {
                drive.drive(0,.6,0);
            }
        }

        // drive forward, farther into storage unit - time dependent on battery voltage
        if (getBatteryVoltage() > 13.6){
            time.reset();
            while (time.seconds() < .35 && opModeIsActive()) {
                drive.drive(.6,0,0);
            }
        } else {
            time.reset();
            while (time.seconds() < .25 && opModeIsActive()) {
                drive.drive(.6,0,0);
            }
        }
    }

    // get battery voltage
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}