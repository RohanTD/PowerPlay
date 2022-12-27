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

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Tilt Auton Left", group = "TestBot")
public class TiltAutonLeft extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL;
    DcMotor liftR;
    DcMotor turret;

    Servo claw;
    Servo extend;
    Servo tilt;

    Pose2d startPose = Constants.startPoseL;
    Pose2d pickupPose = Constants.pickupL;
    Pose2d mainDropPose = Constants.mainDropL;
    Pose2d preCyclePose = Constants.preCycleL;
    Pose2d firstAdjustmentPose = Constants.firstAdjustmentL;
    Pose2d firstDropPose = Constants.firstDropL;

    Vector2d parkLeftPose = Constants.parkLeftL;

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

        claw = Constants.claw;
        extend = Constants.extend;
        tilt = Constants.tilt;

        Constants.setClaw(Constants.ClawPosition.CLOSED);
        tilt.setPosition(Constants.tiltUpPosition);

        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetHigh, Constants.liftPower);
                })
                .addTemporalMarker(0.4, ()->{
                    extend.setPosition(Constants.extendInPos);
                    Constants.setLift(Constants.liftTargetHigh, 0);
                    tilt.setPosition(Constants.tiltDropPosition);
                })
                .addTemporalMarker(0.6,()->{
                    Constants.setTurret(Constants.turretTargetAutonL,true,Constants.turretPower);
                })
                .addTemporalMarker(2.0,()->{
                    Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
                })
                .lineToLinearHeading(Constants.pushOutL)
                .lineToLinearHeading(cutAcrossL)
                .lineToLinearHeading(Constants.altDropL)
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(mainDropPose)
                .lineTo(parkLeftPose)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(startPose);

            drive.followTrajectorySequence(preload);
            Constants.tiltDrop();

            runCycles(drive);

            drive.followTrajectorySequence(parkMiddle);
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
                    .build();

            Constants.setLift(Constants.coneStackHighPosition + (currentCycleCounter * Constants.coneStackInterval),Constants.liftPower);

            drive.followTrajectorySequence(cycle);
            Constants.setClaw(Constants.ClawPosition.CLOSED);
            Constants.sleepTime(300);
            Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
            Constants.sleepTime(200);
            extend.setPosition(Constants.extendInPos);
            Constants.setLift(Constants.liftTargetHigh,0);
            sleep(200);
            Constants.setTurret(Constants.turretTargetAutonL,true,Constants.turretPower);
            drive.followTrajectorySequence(finishCycle);
            Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
            sleep(700);
            Constants.tiltDrop();
            sleep(200);

            cycleCounter++;
        }
    }

}
