package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Open CV Debug", group = "Vision")
public class OpenCVDebug extends LinearOpMode {
    SampleMecanumDrive drive;

    OpenCvCamera phoneCam;
    WebcamName webcamName;
    SignalSleevePipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        Servo camera = hardwareMap.servo.get("camera");
        double cameraPos = Constants.cameraUpPos;

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SignalSleevePipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_up){
                Constants.regionY-= 0.01;
            }
            if (gamepad1.dpad_down){
                Constants.regionY+= 0.01;
            }
            if (gamepad1.dpad_right){
                Constants.regionX+= 0.01;
            }
            if (gamepad1.dpad_left){
                Constants.regionX-= 0.01;
            }

            if (gamepad2.dpad_right){
                Constants.redThresh+= 0.01;
            }
            if (gamepad2.dpad_left){
                Constants.redThresh-=.01;
            }
            if (gamepad2.dpad_up){
                Constants.purpleThresh+=0.01;
            }
            if (gamepad2.dpad_down){
                Constants.purpleThresh-=0.01;
            }

            if (gamepad1.a || gamepad2.a){
                pipeline = new SignalSleevePipeline();
                phoneCam.setPipeline(pipeline);
            }

            cameraPos += gamepad1.right_stick_x * 0.003;
            camera.setPosition(cameraPos);
            if (gamepad1.right_stick_button)
                Constants.cameraUpPos = cameraPos;

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);

            telemetry.addData("Red Threshold", Constants.redThresh);
            telemetry.addData("Purple Threshold", Constants.purpleThresh);

            telemetry.addData("X Coordinate", Constants.regionX);
            telemetry.addData("Y Coordinate", Constants.regionY);

            telemetry.addData("Camera Position", camera.getPosition());

            telemetry.update();
        }
    }


    public static class SignalSleevePipeline extends OpenCvPipeline {
        public enum ParkPosition {
            LEFT,
            CENTER,
            RIGHT
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar PURPLE = new Scalar(255, 0, 255);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar BLACK = new Scalar(0, 0, 0);
        static final Scalar WHITE = new Scalar(255,255,255);

        public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(Constants.regionX, Constants.regionY);

        static final int REGION_WIDTH = 10;
        static final int REGION_HEIGHT = 10;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1;
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

            if (avgCb > Constants.blueThresh) {
                position = ParkPosition.CENTER; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            } else if (avgCr > Constants.redThresh) {
                position = ParkPosition.LEFT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        RED, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            } else {
                position = ParkPosition.RIGHT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        WHITE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            return input;
        }

        public String getAnalysis() {
            return "Y: " + avgY + " Cr: " + avgCr + " Cb: " + avgCb;
        }


    }
}