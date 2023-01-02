package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.Constants.altDropL;
import static org.firstinspires.ftc.teamcode.Constants.cutAcrossL;
import static org.firstinspires.ftc.teamcode.Constants.extend;
import static org.firstinspires.ftc.teamcode.Constants.extendOutPos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.OpenCVDebug;
import org.firstinspires.ftc.teamcode.vision.SignalSleevePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Tilt Auton Left", group = "TestBot")
public class TiltAutonLeft extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL;
    DcMotor liftR;
    DcMotor turret;

    Servo claw;
    Servo extend;
    Servo tilt;
    Servo retraction;
    Servo camera;

    OpenCvCamera phoneCam;
    WebcamName webcamName;
    OpenCVDebug.SignalSleevePipeline pipeline;

    Pose2d startPose = Constants.startPoseL;
    Pose2d pickupPose = Constants.pickupL;
    Pose2d mainDropPose = Constants.mainDropL;
    Pose2d preCyclePose = Constants.preCycleL;
    Pose2d firstAdjustmentPose = Constants.firstAdjustmentL;
    Pose2d firstDropPose = Constants.firstDropL;

    Vector2d parkLeftPose = Constants.parkLeftL;
    Vector2d parkCenterPose = Constants.parkCenterL;
    Vector2d parkRightPose = Constants.parkRightL;

    int numCycles = 5;
    double liftStartTime = 1.3;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Constants.initHardware(hardwareMap);
        Constants.side = Constants.Side.LEFT;

        liftL = Constants.liftL;
        liftR = Constants.liftR;

        turret = Constants.turret;

        camera = Constants.camera;

        claw = Constants.claw;
        extend = Constants.extend;
        tilt = Constants.tilt;
        retraction = Constants.retraction;

        Constants.setClaw(Constants.ClawPosition.CLOSED);
        tilt.setPosition(Constants.tiltUpPosition);
        retraction.setPosition(Constants.retractionDownPos);
        camera.setPosition(Constants.cameraUpPos);

        drive = new SampleMecanumDrive(hardwareMap);

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new OpenCVDebug.SignalSleevePipeline();
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

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetHigh, Constants.liftPower);

                })
                .addTemporalMarker(0.6, ()->{
                    extend.setPosition(Constants.extendInPos);
                    Constants.setLift(Constants.liftTargetHigh, 0);
                    tilt.setPosition(Constants.tiltDropPosition);
                    Constants.setTurret(Constants.turretTargetAutonL,true,0.5);
                })
                .addTemporalMarker(1.3,()->{
                    Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
                })
                .lineToLinearHeading(Constants.pushOutL)
                .lineToLinearHeading(cutAcrossL)
                .lineToLinearHeading(Constants.altDropL)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(altDropL)
                .lineTo(parkLeftPose)
                .build();

        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(altDropL)
                .lineTo(parkCenterPose)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(altDropL)
                .lineTo(parkRightPose)
                .build();

        sleep(2000);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(startPose);

            OpenCVDebug.SignalSleevePipeline.ParkPosition position = pipeline.position;
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            drive.followTrajectorySequence(preload);
            Constants.tiltDrop();

            runCycles(drive);

            if (position == OpenCVDebug.SignalSleevePipeline.ParkPosition.LEFT)
                drive.followTrajectorySequence(parkLeft);
            else if (position == OpenCVDebug.SignalSleevePipeline.ParkPosition.CENTER)
                drive.followTrajectorySequence(parkCenter);
            else
                drive.followTrajectorySequence(parkRight);
            break;
        }
    }

    public void runCycles(SampleMecanumDrive drive){
        int cycleCounter = 0;
        while (cycleCounter < numCycles && opModeIsActive()){
            int currentCycleCounter = cycleCounter;
            TrajectorySequence cycle = drive.trajectorySequenceBuilder(altDropL)
                    .lineToLinearHeading(pickupPose)
                    .addTemporalMarker(Constants.offsetTimePickup, ()->{
                        extend.setPosition(Constants.extendOutPos);
                    })
                    .build();

            TrajectorySequence finishCycle = drive.trajectorySequenceBuilder(pickupPose)
                    .lineToLinearHeading(altDropL)
                    .addTemporalMarker(0.2,()->{
                    })
                    .build();

            Constants.setLift(Constants.coneStackHighPosition + (currentCycleCounter * Constants.coneStackInterval),Constants.liftPower);

            drive.followTrajectorySequence(cycle);
            extend.setPosition(Constants.extendOutPos);
            Constants.setClaw(Constants.ClawPosition.CLOSED);
            Constants.sleepTime(300);
            Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
            sleep(200);
            extend.setPosition(Constants.extendInPos);
            tilt.setPosition(Constants.tiltDropPosition);
            Constants.sleepTime(300);
            Constants.setLift(Constants.liftTargetHigh,0);
            sleep(100);
            Constants.setTurret(Constants.turretTargetAutonL,true,Constants.turretPower);
            Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
            drive.followTrajectorySequence(finishCycle);
            Constants.tiltDrop();

            cycleCounter++;
        }
    }

}
