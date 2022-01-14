package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.objectClasses.LEDStrip;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlinkinTest extends OpMode {
    final RevBlinkinLedDriver.BlinkinPattern LEFT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    final RevBlinkinLedDriver.BlinkinPattern MIDDLE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;
    final RevBlinkinLedDriver.BlinkinPattern RIGHT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE;

    OpenCvCamera webcam;
    ShippingElementDetector detector;
    LEDStrip strip;
    @Override
    public void init() {
        // for live preview in driver station
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "webcam"),
                        cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(352, 288, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        detector = new ShippingElementDetector(352);
        webcam.setPipeline(detector);

        strip = new LEDStrip(hardwareMap, "blinkin", LEDStrip.Alliance.RED);
    }

    @Override
    public void loop() {
        telemetry.addData("Detected Position", detector.location);
        strip.detected(detector.location);
    }
}
