package org.firstinspires.ftc.teamcode.util.Pipelines;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/*
AVERAGE PROCESSING TIMES BASED ON IMAGE RESOLUTION:
320x176 30 ms
800x448 60-80 ms
1280x960 220 ms

This pipeline is designed to filter out the yellow poles and detect their edges.
Additionally, the pipeline also finds the areas and their center of masses of the detected
yellow shapes.

//TODO: make it for not yellow

Once this information is found, the pipeline regularly updates which contour it thinks is the largest,
in other words, which pole is deemed as the "closest"
 */
public class CameraPipeLine extends OpenCvPipeline {
    // for tracking pipeline processing speed
    private final ElapsedTime timer = new ElapsedTime();
    private double processTime = 0;
    // constants
    public static final int CENTER_X = WebcamSubsystem.VIEW_WIDTH / 2;
    public static final int CENTER_Y = WebcamSubsystem.VIEW_HEIGHT / 2;
    private final Size KERNEL = new Size(20, 20);
    private final Scalar WHITE = new Scalar(255, 255, 255);
    private final Scalar CONTOUR_COLOR = new Scalar(255, 255, 255);
    private final Scalar CONTOUR_CENTER_COLOUR = new Scalar(255, 0, 255);
    private final Scalar UPPER_HSV;
    private final Scalar LOWER_HSV;

    // final output image
    Mat output = new Mat();

    private FtcDashboard dashboard;

    public CameraPipeLine(double upperH, double upperS, double upperV,
                          double lowerH, double lowerS, double lowerV,
                          FtcDashboard dashboard) {
        UPPER_HSV = new Scalar(upperH, upperS, upperV);
        LOWER_HSV = new Scalar(lowerH, lowerS, lowerV);
        this.dashboard = dashboard;
    }

    // for contour detection
    private List<Double> contourAreas = new ArrayList<>(); // records contour areas with pixels as the unit
    private double largestContourArea = 0;
    private Point largestContourCenter = new Point(0, 0);

    @Override
    public Mat processFrame(Mat input) {

        if (input.empty()) {
            return input;
        }

        // blurring the input image to improve edge and contour detection
        Mat blur = new Mat();
        Imgproc.blur(input, blur, KERNEL);

        // convert blurred image into HSV scale
        Mat hsv = new Mat();
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);

        // create a HSV threshold that filters out yellow shades
        Mat threshold = new Mat();
        Core.inRange(hsv, LOWER_HSV, UPPER_HSV, threshold);

        Bitmap bm = Bitmap.createBitmap(864, 480, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(threshold, bm);

        dashboard.sendImage(bm);
        // finding contours
        return threshold; // what the camera stream will display on the phone

    }

    /*
     * finds the index of the largest contour and uses that index to grab the desired contour
     * from the list, then it grabs the image moment properties to calculate its enter of mass\
     */
    private void findLargestContourCenter(List<MatOfPoint> contours) {
        MatOfPoint largestContour = contours.get(contourAreas.indexOf(largestContourArea));

        largestContourCenter.x = Imgproc.moments(largestContour).get_m10() / Imgproc.moments(largestContour).get_m00();
        largestContourCenter.y = Imgproc.moments(largestContour).get_m01() / Imgproc.moments(largestContour).get_m00();
    }

    public double getProcessTime() {
        return processTime;
    }

    public List<Double> getContourAreas() {
        return contourAreas;
    }

    public double largestContourArea() {
        return largestContourArea;
    }

    public Point largestContourCenter() {
        return largestContourCenter;
    }

    public boolean objectDetected() {
        return !contourAreas.isEmpty();
    }

}