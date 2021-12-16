package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class DetectElement extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    int width = 352;
    int height = 288;
    // store as variable here so we can access the location
    ShippingElementDetector detector = new ShippingElementDetector(width);

    WebcamName webcamName;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
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
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(detector);
        ShippingElementDetector.ElementLocation location = detector.getLocation();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            telemetry.addData("position", detector.getLocation());
            telemetry.update();
        }
    }
}