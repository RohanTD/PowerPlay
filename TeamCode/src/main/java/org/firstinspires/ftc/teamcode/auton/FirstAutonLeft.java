package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.Constants.extend;


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

@Autonomous(name = "First Auton Left", group = "TestBot")
public class FirstAutonLeft extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL;
    DcMotor liftR;
    DcMotor turretR;

    Servo clawL;
    Servo clawR;
    Servo extend;

    double waitTimeDrop = Constants.waitTimeDrop;
    double waitTimePickup = Constants.waitTimePickup;

    Pose2d startPose = Constants.startPoseL;
    Pose2d pickupPose = Constants.pickupL;
    Pose2d mainDropPose = Constants.mainDropL;
    Pose2d preCyclePose = Constants.preCycleL;
    Pose2d firstAdjustmentPose = Constants.firstAdjustmentL;
    Pose2d firstDropPose = Constants.firstDropL;

    Vector2d parkMiddlePose = Constants.parkMiddleL;

    int numCycles = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Constants.initHardware(hardwareMap);

        liftL = Constants.liftL;
        liftR = Constants.liftR;

        turretR = Constants.turretR;

        clawL = Constants.clawL;
        clawR = Constants.clawR;

        extend = Constants.extend;

        Constants.setClaw(Constants.ClawPosition.CLOSED);

        extend.setPosition(Constants.extendOutPos);
        sleep(500);
        extend.setPosition(Constants.extendInPos);

        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0.01, ()->{
                    Constants.setClaw(Constants.ClawPosition.CLOSED);
                    Constants.setLift(-2900,Constants.liftPower);
                    Constants.setTurret(1450,true,Constants.turretPower);
                })
                .lineToLinearHeading(firstAdjustmentPose)
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

            drive.followTrajectorySequence(preload);

            extend.setPosition(Constants.extendBackPos);
            sleep(500);
            Constants.setClaw(Constants.ClawPosition.OPEN);
            sleep(200);
            extend.setPosition(Constants.extendInPos);
            Constants.setLift(0,Constants.liftPower);
            Constants.setTurret(0,true,Constants.turretPower);
            sleep(200);

            drive.followTrajectorySequence(cycleSetup);

            runCycles(drive);

//            drive.followTrajectorySequence(parkMiddle);

            break;
        }
    }

    public void runCycles(SampleMecanumDrive drive){
        int cycleCounter = 0;
        while (cycleCounter < numCycles && opModeIsActive()){
            int currentCycleCounter = cycleCounter;
            TrajectorySequence cycle = drive.trajectorySequenceBuilder(mainDropPose)
                    .lineToLinearHeading(pickupPose)
                    .addTemporalMarker(0.3,()->{
                        extend.setPosition(Constants.extendOutPos);
                        Constants.setLift(Constants.coneStackHighPosition + (currentCycleCounter * Constants.coneStackInterval),0.7);
                    })
                    .build();

            TrajectorySequence finishCycle = drive.trajectorySequenceBuilder(Constants.LSecond)
                    .lineToLinearHeading(mainDropPose)
                    .build();

            drive.followTrajectorySequence(cycle);
            Constants.setClaw(Constants.ClawPosition.CLOSED);
            sleep(500);
            Constants.setLift(-2950,Constants.liftPower);
            sleep(200);
            extend.setPosition(Constants.extendInPos);
            sleep(500);
            Constants.setTurret(-740,true,Constants.turretPower);
            drive.followTrajectorySequence(finishCycle);
            sleep(100);
            extend.setPosition(Constants.extendRightPos);
            sleep(400);
            Constants.setClaw(Constants.ClawPosition.OPEN);
            sleep(200);
            extend.setPosition(Constants.extendInPos);
            Constants.setLift(0,Constants.liftPower);
            Constants.setTurret(0,true,Constants.turretPower);
            sleep(200);
            cycleCounter++;
        }
    }

}
