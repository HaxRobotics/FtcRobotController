package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled
public class BlinkinTestContour extends OpMode {
    final RevBlinkinLedDriver.BlinkinPattern LEFT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    final RevBlinkinLedDriver.BlinkinPattern MIDDLE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    final RevBlinkinLedDriver.BlinkinPattern RIGHT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    final RevBlinkinLedDriver.BlinkinPattern NONE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;

    OpenCvCamera webcam;
    ContourPipeline pipeline;
    RevBlinkinLedDriver blinkin;
    int minRectangleArea = 2000;
    double rightBarcodeRangeBoundary = 0.6;
    double leftBarcodeRangeBoundary = 0.3;
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
        pipeline = new ContourPipeline(0,0,0,0);
        webcam.setPipeline(pipeline);


        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

    }

    @Override
    public void loop() {
        double rectangleArea = pipeline.getRectArea();

        //Print out the area of the rectangle that is found.
        telemetry.addData("Rectangle Area", rectangleArea);
        if(rectangleArea > minRectangleArea){
            //Then check the location of the rectangle to see which barcode it is in.
            if(pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()){
                telemetry.addData("Barcode Position", "Right");
                blinkin.setPattern(RIGHT_PATTERN);
            }
            else if(pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()){
                telemetry.addData("Barcode Position", "Left");
                blinkin.setPattern(LEFT_PATTERN);
            }
            else {
                telemetry.addData("Barcode Position", "Center");
                blinkin.setPattern(MIDDLE_PATTERN);
            }
        }
    }
}
