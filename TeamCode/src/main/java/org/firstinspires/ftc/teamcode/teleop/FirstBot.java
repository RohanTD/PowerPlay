package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "PowerPlay First Bot", group = "TestBot")
public class FirstBot extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL = null;
    DcMotor liftR = null;

    Servo clawL = null;
    Servo clawR = null;
    Servo extend = null;

    DcMotor turretL = null;
    DcMotor turretR = null;

    Gamepad g1 = gamepad1;
    Gamepad g2 = gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        g1 = gamepad1;
        g2 = gamepad2;

        liftL = hardwareMap.dcMotor.get("lift_right");
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR = hardwareMap.dcMotor.get("lift_left");
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        turretL = hardwareMap.dcMotor.get("turret_left");
//        turretL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretR = hardwareMap.dcMotor.get("turret_right");
        turretR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawL = hardwareMap.servo.get("left_claw");
        clawR = hardwareMap.servo.get("right_claw");

        extend = hardwareMap.servo.get("extend");

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            teleopCode();
            teleopAuto();
        }
    }

    public void teleopAuto() {
        if (g2.y && Math.abs(liftL.getCurrentPosition() - Constants.liftTargetHigh) >= Constants.liftError) {
            Constants.setLift(Constants.liftTargetHigh, Constants.liftPower);
        }
        if (g2.a && Math.abs(liftL.getCurrentPosition()) >= Constants.liftError) {
            Constants.setLift(0, Constants.liftPower);
        }
        if (!g2.a && !g2.y) {
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double liftInput = Math.pow(gamepad2.right_stick_y, 2);
            if (gamepad2.right_stick_y < 0)
                liftInput *= -1;

            liftL.setPower(liftInput);
            liftR.setPower(-liftInput);
        }

        if (g2.dpad_right && Math.abs(turretR.getCurrentPosition() - Constants.turretTarget90) >= Constants.turretError) {
            Constants.setTurret(90, false, Constants.turretPower);
        }
        if (g2.dpad_left && Math.abs(turretR.getCurrentPosition() - Constants.turretTargetNeg90) >= Constants.turretError) {
            Constants.setTurret(-90, false, Constants.turretPower);
        }
        if (g2.dpad_down && Math.abs(turretR.getCurrentPosition() - Constants.turretTarget180) >= Constants.turretError) {
            Constants.setTurret(180, false, Constants.turretPower);
        }
        if (g2.dpad_up && Math.abs(turretR.getCurrentPosition()) >= Constants.turretError) {
            Constants.setTurret(0, false, Constants.turretPower);
        }
        if (!g2.dpad_left && !g2.dpad_down && !g2.dpad_up && !g2.dpad_right) {
            double turretInput = -Math.pow(gamepad2.left_stick_x, 2) * 0.5;
            if (g2.left_trigger == 1)
                turretInput *= 0.5;
            if (gamepad2.left_stick_x < 0)
                turretInput *= -1;

//        turretL.setPower(turretInput);
            turretR.setPower(turretInput);
        }


        if (g2.b) {
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turretR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void teleopCode() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        double multiplier = 0.65;
        double power = 2.0;

        if (g1.right_trigger == 1) {
            multiplier = 0.4;
        }
        if (g1.left_trigger == 1) {
            multiplier = 1;
        }

        double yMult = multiplier;
        double xMult = multiplier;
        double turnMult = multiplier;

        if (y < 0)
            yMult *= -1;
        if (x < 0)
            xMult *= -1;
        if (turn < 0)
            turnMult *= -1;

        y = Math.pow(y, power) * yMult;
        x = Math.pow(x, power) * xMult;
        turn = Math.pow(turn, power) * turnMult;

        drive.setWeightedDrivePower(new Pose2d(y, x, turn));

        if (gamepad2.right_bumper) {
            Constants.setClaw(Constants.ClawPosition.CLOSED);
        }
        if (gamepad2.left_bumper) {
            Constants.setClaw(Constants.ClawPosition.OPEN);
        }

        extend.setPosition(gamepad2.right_trigger * (Constants.extendOutPos - Constants.extendInPos) + Constants.extendInPos);

        telemetry.addData("Left lift position", liftL.getCurrentPosition());
        telemetry.addData("Right lift position", liftR.getCurrentPosition());
        telemetry.addData("Turret position", turretR.getCurrentPosition());
        telemetry.update();
    }
}
