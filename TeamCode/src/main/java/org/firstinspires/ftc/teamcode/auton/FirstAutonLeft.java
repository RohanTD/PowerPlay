package org.firstinspires.ftc.teamcode.auton;

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

    double waitTimeDrop = Constants.waitTimeDrop;
    double waitTimePickup = Constants.waitTimePickup;

    Pose2d startPose = Constants.startPoseL;
    Pose2d pickupPose = Constants.pickupL;
    Pose2d mainDropPose = Constants.mainDropL;
    Pose2d preCyclePose = Constants.preCycleL;
    Pose2d firstAdjustmentPose = Constants.firstAdjustmentL;
    Pose2d firstDropPose = Constants.firstDropL;

    Vector2d parkMiddlePose = Constants.parkMiddleL;

    int cycleCounter = 1;

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

        Constants.setClaw(Constants.ClawPosition.CLOSED);

        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(0.01, Constants.prepareArmB)
                .lineToLinearHeading(firstAdjustmentPose)
                .lineToLinearHeading(firstDropPose)
                .addDisplacementMarker(Constants.resetArm)
                .lineToLinearHeading(preCyclePose)
                .build();

        TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(mainDropPose)
                .lineTo(parkMiddlePose)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setPoseEstimate(startPose);

            drive.followTrajectorySequence(preload);

            runCycles(drive);

            drive.followTrajectorySequence(parkMiddle);

            break;
        }
    }

    public void runCycles(SampleMecanumDrive drive){
        while (cycleCounter > 0){
            TrajectorySequence cycle = drive.trajectorySequenceBuilder(preCyclePose)
                    .lineToLinearHeading(pickupPose)
                    .addDisplacementMarker(Constants.prepareArmB)
                    .lineToLinearHeading(mainDropPose)
                    .addDisplacementMarker(Constants.resetArm)
                    .build();
            drive.followTrajectorySequence(cycle);
            cycleCounter--;
        }
    }

}
//lol
