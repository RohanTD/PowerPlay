package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.Constants.extend;
import static org.firstinspires.ftc.teamcode.Constants.extendOutPos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "First Auton Left", group = "TestBot")
public class FirstAutonLeft extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL;
    DcMotor liftR;
    DcMotor turretR;

    Servo claw;
    Servo extend;

    Pose2d startPose = Constants.startPoseL;
    Pose2d pickupPose = Constants.pickupL;
    Pose2d mainDropPose = Constants.mainDropL;
    Pose2d preCyclePose = Constants.preCycleL;
    Pose2d firstAdjustmentPose = Constants.firstAdjustmentL;
    Pose2d firstDropPose = Constants.firstDropL;

    Vector2d parkMiddlePose = Constants.parkLeftL;

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

        turretR = Constants.turret;

        claw = Constants.claw;

        extend = Constants.extend;

        Constants.setClaw(Constants.ClawPosition.OPEN);

//        extend.setPosition(Constants.extendOutPos);
//        sleep(500);
//        extend.setPosition(Constants.extendInPos);

        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.1, ()->{
                    Constants.setLift(Constants.liftTargetHigh, Constants.liftPower);
                })
                .addTemporalMarker(0.4, ()->{
                    extend.setPosition(Constants.extendInPos);
                    Constants.setLift(Constants.liftTargetHigh, 0);
                })
                .addTemporalMarker(0.5,()->{
                    Constants.setTurret(Constants.turretTargetNeg90,true,Constants.turretPower);
                })
                .addTemporalMarker(0.7, ()->{
                    Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
                })
                .lineToLinearHeading(firstAdjustmentPose)
                .addDisplacementMarker(()->{
                    extend.setPosition(Constants.extendBackPos);
                })
                .UNSTABLE_addDisplacementMarkerOffset(3,()->{
                    Constants.setTurret(Constants.turretTarget180,true,Constants.turretPower);
                })
                .lineToLinearHeading(firstDropPose)
                .build();

        TrajectorySequence cycleSetup = drive.trajectorySequenceBuilder(firstDropPose)
                .lineToLinearHeading(preCyclePose)
                .lineToLinearHeading(mainDropPose)
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(mainDropPose)
                .lineTo(parkMiddlePose)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(startPose);

            extend.setPosition(0.9);
            sleep(200);
            Constants.setClaw(Constants.ClawPosition.CLOSED);
            sleep(300);
            drive.followTrajectorySequence(preload);

            Constants.dropAndReset();

            drive.followTrajectorySequence(cycleSetup);

            runCycles(drive);

            drive.followTrajectorySequence(parkMiddle);

            break;
        }
    }

    public void runCycles(SampleMecanumDrive drive){
        int cycleCounter = 0;
        while (cycleCounter < numCycles && opModeIsActive()){
            int currentCycleCounter = cycleCounter;
            TrajectorySequence cycle = drive.trajectorySequenceBuilder(mainDropPose)
                    .lineToLinearHeading(pickupPose)
                    .addTemporalMarker(Constants.extendOffset, ()->{
                        Constants.setClaw(Constants.ClawPosition.CLOSED);
                    })
                    .build();

            TrajectorySequence finishCycle = drive.trajectorySequenceBuilder(pickupPose)
                    .lineToLinearHeading(new Pose2d(mainDropPose.getX() /*+ (currentCycleCounter - 2) * 0.5*/, mainDropPose.getY(), mainDropPose.getHeading()))
                    .addTemporalMarker(0.2,()->{
                        Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
                    })
                    .addTemporalMarker(1.0,()->{
                        int turretTarget = Constants.turretTarget90;
                        if (Constants.side == Constants.Side.RIGHT)
                            turretTarget = Constants.turretTargetNeg90;
                        Constants.setTurret(turretTarget,true,Constants.turretPower);

                    })
                    .addTemporalMarker(1.4, ()->{
                        extend.setPosition(Constants.extendRightPos);
                    })
                    .build();

            extend.setPosition(Constants.extendOutPos);
            Constants.setLift(Constants.coneStackHighPosition + (currentCycleCounter * Constants.coneStackInterval),Constants.liftPower);

            drive.followTrajectorySequence(cycle);
            Constants.setClaw(Constants.ClawPosition.CLOSED);
//            Constants.sleepTime(100);
            Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
            Constants.sleepTime(200);
            extend.setPosition(Constants.extendInPos);
            Constants.setLift(Constants.liftTargetHigh,0);

            drive.followTrajectorySequence(finishCycle);
            extend.setPosition(Constants.extendRightPos);
            sleep(150);
            Constants.dropAndReset();
            sleep(150);

            cycleCounter++;
        }
    }

}
