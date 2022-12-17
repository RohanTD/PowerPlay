package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalSleevePipeline extends OpenCvPipeline {
    public enum ParkPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar PURPLE = new Scalar(255, 0, 255);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLACK = new Scalar(0,0,0);

    public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(250, 90);

    static final int REGION_WIDTH = 10;
    static final int REGION_HEIGHT = 10;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat region1, region2, region3;
    Mat YCrCb = new Mat();

    int avgY, avgCr, avgCb;

    public volatile ParkPosition position = ParkPosition.CENTER;

    void inputToCr(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCr(input);

        region1 = YCrCb.submat(new Rect(region1_pointA, region1_pointB));

        avgY = (int) Core.mean(region1).val[0];
        avgCr = (int) Core.mean(region1).val[1];
        avgCb = (int) Core.mean(region1).val[2];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLACK, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        if (avgCr > 100 && avgCb > 100)
        {
            position = ParkPosition.RIGHT; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    PURPLE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

        } else if (avgCr > 170)
        {
            position = ParkPosition.LEFT; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

        } else
        {
            position = ParkPosition.CENTER; // Record our analysis

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        return input;
    }

    public String getAnalysis() {
        return "Y: " + avgY + " Cr: " + avgCr + " Cb: " + avgCb;
    }


}