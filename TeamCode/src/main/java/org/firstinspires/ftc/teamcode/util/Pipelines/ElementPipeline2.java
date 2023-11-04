package org.firstinspires.ftc.teamcode.util.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


//-------------------------------------------------------------------------------------------------------------------------------

//EXAMPLE

//-------------------------------------------------------------------------------------------------------------------------------


public class ElementPipeline2 extends OpenCvPipeline {

    public enum ElementPosition {
        p1, //b
        p2, //g
        p3, //r
        none
    }

    final Scalar Border = new Scalar(0,255,0);
    final Scalar Fill = new Scalar(0,0,255);

    static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(0,185);
    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 40;

    Point region_pointA = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y);
    Point region_pointB = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat regionR;
    Mat regionG;
    Mat regionB;
    Mat rgb = new Mat();
    Mat r = new Mat();
    Mat g = new Mat();
    Mat b = new Mat();
    int avgR;
    int avgG;
    int avgB;

    public ElementPosition position = null;

    void inputToRGB(Mat input){
        rgb = input;
        Core.extractChannel(rgb, r, 0);
        Core.extractChannel(rgb, g, 1);
        Core.extractChannel(rgb, b, 2);

    }

    @Override
    public void init(Mat firstFrame)
    {
            /* init method takes the firstFrame and crops down to focus on the rectangular space over the frame
            arguments:
            -- Mat firstFrame is, as implied, the first frame taken
             */


    }

    @Override
    public Mat processFrame(Mat input) {
/* processFrame method processes the inputs and sets position to equal correct number of rings
            arguments:
            -- Mat input is the image taken from the webcam
             */

        inputToRGB(input);
        Rect area = new Rect(region_pointA, region_pointB);
        regionR = r.submat(area);
        regionG = g.submat(area);
        regionB = b.submat(area);

        //COMMENT: Convert to Cb
        //  inputToCb(input);

        //COMMENT: Draw the rectangle (Blue)
        Imgproc.rectangle(
                input, //Buffer to draw on
                region_pointA, //First point which defines the rectangle
                region_pointB, //Second point which defines the rectangle
                Border, //Colour rectangle blue
                2);

        //COMMENT: Amount of orange (ring) in the image analyzed
        avgR = (int) Core.mean(regionR).val[0];
        avgG = (int) Core.mean(regionG).val[0];
        avgB = (int) Core.mean(regionB).val[0];

        //COMMENT: Set the ring positions
        // Record our analysis

        int result = Math.max(avgR, Math.max(avgB, avgG));
        if (result == avgB){
            position = ElementPosition.p2;
        } else if (result == avgG){
            position = ElementPosition.p3;
        } else {
            position = ElementPosition.p1;
        }

        //COMMENT: Draw the rectangle (Green)

        Imgproc.rectangle(
                input, // Buffer to draw on
                region_pointA, // First point which defines the rectangle
                region_pointB, // Second point which defines the rectangle
                Fill, // The color the rectangle is drawn in
                -1);

        return input;
    }

    public int getAnalysisR() {
        return avgR;
    }
    public int getAnalysisG() {
        return avgG;
    }
    public int getAnalysisB() {
        return avgB;
    }
}
