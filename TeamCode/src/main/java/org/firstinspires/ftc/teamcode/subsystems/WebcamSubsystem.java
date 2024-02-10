package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.util.Pipelines.ContourPipeline;
import org.firstinspires.ftc.teamcode.util.Pipelines.ElementPipeline2;
import org.firstinspires.ftc.teamcode.util.Pipelines.RedIntakePipeline;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.nio.channels.Pipe;

public class WebcamSubsystem extends Specifications {
    public OpenCvCamera webcam;
    int cameraMonitorViewId;
    ElementPipeline2 elementPipeline2;
    RedIntakePipeline redIntakePipeline;
    //public AprilTagPipeline aprilTagPipeline;
    public ContourPipeline contourPipeline;
    boolean initial;

    public enum PipelineName {
        ELEMENT, RED_INTAKE, APRIL_TAG, CONTOUR_RED, CONTOUR_BLUE
    }

    public static final int VIEW_WIDTH = 864;
    public static final int VIEW_HEIGHT = 480;
    public static final int CENTER_X = VIEW_WIDTH / 2;
    public static final int CENTER_Y = VIEW_HEIGHT / 2;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    public WebcamSubsystem(HardwareMap hardwareMap, PipelineName pipelineName) {
        contourPipeline = new ContourPipeline(30, 255, 255, 10, 130, 130);
//        if(pipelineName == PipelineName.APRIL_TAG){
//            aprilTagPipeline = new AprilTagPipeline(0.166, fy, fx, cy, cx);
//            webcam.setPipeline(aprilTagPipeline);
//        }
//        else if(pipelineName == PipelineName.ELEMENT){
//            elementPipeline2 = new ElementPipeline2();
//            webcam.setPipeline(elementPipeline2);
//        }
//        else if(pipelineName == PipelineName.RED_INTAKE){
//            redIntakePipeline = new RedIntakePipeline();
//            webcam.setPipeline(redIntakePipeline);
//        }
        if (pipelineName == PipelineName.CONTOUR_BLUE) {
            contourPipeline = new ContourPipeline(87, 218, 143, 138, 255, 255);
        } else if (pipelineName == PipelineName.CONTOUR_RED) {
            contourPipeline = new ContourPipeline(0, 202, 108, 26, 255, 255);
        }
        // initiate the needed parameters
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // initiate the camera object with created parameters and pipeline
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(contourPipeline);
        // runs camera on a separate thread so it can run simultaneously with everything else

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // starts the camera stream when init is pressed
                /*
                 When the camera turns on, we start streaming data,
                 sending frames through the VisionPortal.

                 Note, the camera stream starts when init is pressed.
                 */
                webcam.startStreaming(VIEW_WIDTH, VIEW_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void switchPipeline() {
        redIntakePipeline = new RedIntakePipeline();
        webcam.setPipeline(redIntakePipeline);
        webcam.setPipeline(null);
    }

    public ElementPipeline2.ElementPosition position2() {
        return elementPipeline2.position;
    }

    public float getX() {
        return redIntakePipeline.getXPosition();
    }

    public float getY() {
        return redIntakePipeline.getYTopPosition();
    }

    public int getAnalysisR() {
        return elementPipeline2.getAnalysisR();
    }

    public int getAnalysisG() {
        return elementPipeline2.getAnalysisG();
    }

    public int getAnalysisB() {
        return elementPipeline2.getAnalysisB();
    }

    public void stopCamera() {
        webcam.stopStreaming();
    }

    public double getXProp() {
        return contourPipeline.largestContourCenter().x;
    }
}