package org.firstinspires.ftc.teamcode.auton;

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

@Autonomous(name = "Super Fast", group = "TestBot")
public class SuperFast extends LinearOpMode {
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
    Pose2d altDropPose = Constants.altDropL;

    Pose2d pushOutPose = Constants.pushOutL;
    Pose2d cutAcrossPose = Constants.cutAcrossL;

    Vector2d parkLeftPose = Constants.parkLeftL;
    Vector2d parkCenterPose = Constants.parkCenterL;
    Vector2d parkRightPose = Constants.parkRightL;

    int numCycles = 5;

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

        claw = Constants.claw;
        extend = Constants.extend;
        tilt = Constants.tilt;

        retraction = Constants.retraction;
        camera = Constants.camera;

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
            public void onError(int errorCode) { }
        });

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(pushOutPose)
                .lineToLinearHeading(cutAcrossPose)
                .lineToLinearHeading(altDropPose)
                .addTemporalMarker(Constants.preparePreloadOffset, Constants.preparePreload)
                .addTemporalMarker(Constants.finishLiftPreloadOffset, () -> {
                    Constants.setLift(Constants.liftTargetHigh, Constants.liftPower);
                })
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(altDropPose)
                .lineTo(parkLeftPose)
                .build();

        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(altDropPose)
                .lineTo(parkCenterPose)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(altDropPose)
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
//            sleep(200);

            Constants.setLift(Constants.liftTargetHigh, Constants.liftPower);
            drive.followTrajectorySequence(preload);
            extend.setPosition(Constants.extendOutPos);
            tilt.setPosition(Constants.tiltDropPosition);
            Constants.sleepTime(300);
            tilt.setPosition(Constants.tiltDownPosition);
            sleep(100);
            Constants.setClaw(Constants.ClawPosition.OPEN);
            extend.setPosition(Constants.extendInPos);
            sleep(200);
            Constants.setTurret(0,true,Constants.turretPower);
            Constants.setLift(Constants.coneStackHighPosition,Constants.liftPower);

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

    public void runCycles(SampleMecanumDrive drive) {
        int cycleCounter = 0;
        while (cycleCounter < numCycles && opModeIsActive()) {
            int currentCycleCounter = cycleCounter;
            TrajectorySequence cycle = drive.trajectorySequenceBuilder(altDropPose)
                    .lineToLinearHeading(pickupPose)
                    .addTemporalMarker(0.6, () -> {
                        extend.setPosition(Constants.extendOutPos);
                    })
                    .build();

            TrajectorySequence finishCycle = drive.trajectorySequenceBuilder(pickupPose)
                    .lineToLinearHeading(altDropPose)
                    .addTemporalMarker(0.7, ()->{
                        extend.setPosition(Constants.extendOutPos);
                    })
                    .build();

            Constants.setLift(Math.min(Constants.coneStackHighPosition + (currentCycleCounter * Constants.coneStackInterval), 0), Constants.liftPower);

            drive.followTrajectorySequence(cycle);
            extend.setPosition(Constants.extendOutPos);
            Constants.setClaw(Constants.ClawPosition.CLOSED);
            Constants.sleepTime(200);

            Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
            Constants.sleepTime(200);
            extend.setPosition(Constants.extendInPos);
            tilt.setPosition(Constants.tiltDropPosition);
            int turretTarget = Constants.turretTargetAutonL;
            if (Constants.side == Constants.Side.RIGHT)
                turretTarget = Constants.turretTargetAutonR;
            Constants.setTurret(turretTarget,true,Constants.turretPower);

            drive.followTrajectorySequence(finishCycle);
            tilt.setPosition(Constants.tiltDropPosition);
            extend.setPosition(Constants.extendOutPos);
//        Constants.sleepTime(100);
            tilt.setPosition(Constants.tiltDownPosition);
            Constants.sleepTime(100);
            Constants.setClaw(Constants.ClawPosition.OPEN);
            extend.setPosition(Constants.extendInPos);
            sleep(200);
            Constants.setTurret(0,true,Constants.turretPower);
            Constants.setLift(Constants.coneStackHighPosition,Constants.liftPower);

            cycleCounter++;
        }
    }

}
