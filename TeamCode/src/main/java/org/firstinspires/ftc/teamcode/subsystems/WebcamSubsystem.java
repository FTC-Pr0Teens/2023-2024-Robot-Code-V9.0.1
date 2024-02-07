package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ContourProcessor;
import org.firstinspires.ftc.teamcode.util.Pipelines.ElementPipeline2;
import org.firstinspires.ftc.teamcode.util.Pipelines.RedIntakePipeline;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class WebcamSubsystem extends Specifications {
    public OpenCvCamera webcam;
    ElementPipeline2 elementPipeline2;
    RedIntakePipeline redIntakePipeline;
    boolean initial;
    public enum PipelineName{
        ELEMENT, RED_INTAKE, APRIL_TAG, CONTOUR_RED, CONTOUR_BLUE
    }

    public static final int VIEW_WIDTH = 320;
    public static final int VIEW_HEIGHT = 176;
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

    private final VisionPortal VISION_PORTAL;
    private final ContourProcessor CONTOUR_PROCESSOR;
    private final AprilTagProcessor APRIL_TAG_PROCESSOR;
    /**
     * Createss a usable WebcamSubSystem object
     * @param hardwareMap
     * @param pipelineName
     */
    public WebcamSubsystem(HardwareMap hardwareMap, PipelineName pipelineName){

        //The aprilTagProcessor is used to find april tags
        APRIL_TAG_PROCESSOR = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
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

        /*
        The contourProcessor is used to find the outines/contours of the spike game pieces

        Also, these upper lower hsv values seem important, so I'm keeping them here
        30, 255, 255,
        10, 130, 130
         */
        if(pipelineName == PipelineName.CONTOUR_BLUE){
            CONTOUR_PROCESSOR = new ContourProcessor(
                    115, 234, 255,
                    104, 130, 41
            );
        } else {
            CONTOUR_PROCESSOR = new ContourProcessor(
                    10, 255, 255,
                    0, 119, 0
            );
        }

        /*
         The visionPortal takes data from a webcam,
         and feeds it into processors that we've attached to it.

         The processors then take these frame,
         and do whatever it is they do.
         */
        VISION_PORTAL = new VisionPortal.Builder()
                .addProcessor(APRIL_TAG_PROCESSOR)
                .addProcessor(CONTOUR_PROCESSOR)
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .setCameraResolution(new Size(864, 480))
                .build();


        // Creating the OpenCvCamera object to get data from
        webcam = OpenCvCameraFactory.getInstance().createWebcam(

                // The Webcam's name
                hardwareMap.get(WebcamName.class, "Webcam 1"),

                // The cameraMonitorViewId value
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId","id",hardwareMap.appContext.getPackageName()
                )
        );

        // Turns on the camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                /*
                 When the camera turns on, we start streaming data,
                 sending frames through the VisionPortal.

                 Note, the camera stream starts when init is pressed.
                 */
                webcam.startStreaming(VIEW_WIDTH,VIEW_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void switchPipeline(){
        redIntakePipeline = new RedIntakePipeline();
        webcam.setPipeline(redIntakePipeline);
        webcam.setPipeline(null);
    }

    public ElementPipeline2.ElementPosition position2(){
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

    public void stopCamera(){
        webcam.stopStreaming();
    }

    public double getXProp(){
        return CONTOUR_PROCESSOR.largestContourCenter().x;
    }

    public ContourProcessor getContourProcessor() {
        return CONTOUR_PROCESSOR;
    }

    /**
     * Identifies what spike location the game piece is on.
     *
     * @return a String indication the location of the game piece
     */
    public String findSpikePosition() {
        // For reference, camera is 864 pixels wide
        double center = CONTOUR_PROCESSOR.largestContourCenter().x;
        return center < 288 ? "left"
                : center < 576 ? "middle"
                : "right";
    }

    //returns list of all april tag detections

    /**
     * Returns a list of all detected April Tag IDs.
     *
     * @return an integer arraylist containing the detected April Tag IDs
     */
    public ArrayList<Integer> getDetections(){
        ArrayList<Integer> aprilTagIDs = new ArrayList<>();

        // This loop formats the detections into data we can actually read
        for (AprilTagDetection det : APRIL_TAG_PROCESSOR.getDetections())
            aprilTagIDs.add(det.id);

        return aprilTagIDs;
    }
    public VisionPortal.CameraState getCameraState() {
        return VISION_PORTAL.getCameraState();
    }
}
