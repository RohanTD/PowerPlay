package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(name = "Coordinate Automation", group = "TestBot")
public class CoordinateAutomation extends LinearOpMode {
    SampleMecanumDriveCancelable drive;

    Gamepad g1 = gamepad1;
    Gamepad g2 = gamepad2;

    enum Mode {
        DRIVER,
        AUTO
    }

    Mode currentMode = Mode.DRIVER;
    Pose2d poseEstimate;
    Pose2d targetPose = new Pose2d();

    boolean isDown = false;
    int currentCount = 0;
    int storedCount = 0;

    double robotMidDisplacement = 9;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        poseEstimate = new Pose2d();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            asyncTeleop();
        }
    }

    public void driveCode(){
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        double multiplier = 0.6;
        drive.setWeightedDrivePower(new Pose2d(multiplier * y, multiplier * x, (multiplier) * turn));

        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            drive.setPoseEstimate(new Pose2d(0, robotMidDisplacement, 0));
        }

        if (gamepad1.right_trigger == 1) {
            currentCount = isDown ? currentCount : 0;
            isDown = true;

            if (g1.dpad_right) currentCount += 5;
            if (g1.dpad_left) currentCount -= 5;
            if (g1.dpad_up) currentCount += 1;
            if (g1.dpad_down) currentCount -= 1;

            if (currentCount > 25) currentCount -= 25;
            if (currentCount < 0) currentCount += 25;

            if (g1.x){
                currentCount = 0;
            }

        } else if (isDown && currentCount > 0) {
            isDown = false;
            storedCount = currentCount;
            drive.followTrajectorySequenceAsync(generateTrajectory(storedCount));

            currentMode = Mode.AUTO;
        }
    }

    public void armCode() {
    }

    public TrajectorySequence generateTrajectory(int junction) {
        int column = junction % 5;
        double targetX = (24 - robotMidDisplacement) + (column == 3 ? -6 : Math.abs(2 - column) * 24);
        double targetY = (24 - robotMidDisplacement) + 24 * (junction / 5);

        if (column < 3) targetX *= -1;

        targetPose = new Pose2d(targetX, targetY, 0);

        return drive.trajectorySequenceBuilder(poseEstimate)
                .lineToLinearHeading(new Pose2d(targetX, (24 - robotMidDisplacement * 2), 0))
                .lineToLinearHeading(targetPose)
                .build();
    }

    public void asyncTeleop() {
        drive.update();

        poseEstimate = drive.getPoseEstimate();

        int digits = 2;

        telemetry.addData("Mode", currentMode);
        telemetry.addData("X", Math.round(poseEstimate.getX() * Math.pow(10, digits)) / Math.pow(10, digits));
        telemetry.addData("Y", Math.round(poseEstimate.getY() * Math.pow(10, digits)) / Math.pow(10, digits));
        telemetry.addData("Heading", Math.round(poseEstimate.getHeading() * Math.pow(10, digits)) / Math.pow(10, digits));
        telemetry.addData("Junction", storedCount);
        telemetry.addData("Target", targetPose);
        telemetry.update();

        switch (currentMode) {
            case DRIVER:
                driveCode();
                armCode();
                break;
            case AUTO:
                if (gamepad1.b || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_y !=0){
                    storedCount = 0;
                    drive.breakFollowing();
                    currentMode = Mode.DRIVER;
                }
                if (!drive.isBusy()) {
                    storedCount = 0;
                    currentMode = Mode.DRIVER;
                }
                armCode();
                break;
        }
    }
}
