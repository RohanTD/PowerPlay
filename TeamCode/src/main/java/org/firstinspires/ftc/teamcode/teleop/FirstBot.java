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
        // when you hold y, the lift will move upward until it hits the target position (liftTargetHigh)
        if (g2.y && Math.abs(liftL.getCurrentPosition() - Constants.liftTargetHigh) >= Constants.liftError) {
            Constants.setLift(Constants.liftTargetHigh, Constants.liftPower); // you can see this method in Constants
        }
        // when you hold a, the lift will move downward until it gets down to 0
        if (g2.a && Math.abs(liftL.getCurrentPosition()) >= Constants.liftError) {
            Constants.setLift(0, Constants.liftPower);
        }
        // if neither a nor y are pressed, the right joystick will be controlling lift
        if (!g2.a && !g2.y) {
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // squared input
            double liftInput = Math.pow(gamepad2.right_stick_y, 2);
            if (gamepad2.right_stick_y < 0) liftInput *= -1;

            // opposite powers
            liftL.setPower(liftInput);
            liftR.setPower(-liftInput);
        }

        //Dpad right -> turret goes to 90 degrees (right)
        if (g2.dpad_right && Math.abs(turretR.getCurrentPosition() - Constants.turretTarget90) >= Constants.turretError) {
            Constants.setTurret(90, false, Constants.turretPower); // look at this method in Constants
        }
        //Dpad left -> turret goes to -90 degrees (left)
        if (g2.dpad_left && Math.abs(turretR.getCurrentPosition() - Constants.turretTargetNeg90) >= Constants.turretError) {
            Constants.setTurret(-90, false, Constants.turretPower);
        }
        //Dpad down -> turret goes to 180 degrees (backward)
        if (g2.dpad_down && Math.abs(turretR.getCurrentPosition() - Constants.turretTarget180) >= Constants.turretError) {
            Constants.setTurret(180, false, Constants.turretPower);
        }
        //Dpad up -> turret goes to 0 degrees (forward)
        if (g2.dpad_up && Math.abs(turretR.getCurrentPosition()) >= Constants.turretError) {
            Constants.setTurret(0, false, Constants.turretPower);
        }
        //If no dpads are pressed, left joystick will control turret
        if (!g2.dpad_left && !g2.dpad_down && !g2.dpad_up && !g2.dpad_right) {
            // squared input, 0.5 speed
            double turretInput = -Math.pow(gamepad2.left_stick_x, 2) * 0.5;
            // left trigger = slow mode
            if (g2.left_trigger == 1) turretInput *= 0.5;
            if (gamepad2.left_stick_x < 0) turretInput *= -1;

//        turretL.setPower(turretInput);
            turretR.setPower(turretInput);
        }

        // press b -> resets all encoders
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

        // only important numbers are here - default movement speed
        double multiplier = 0.65;
        // curve to apply (squared rn)
        double power = 2.0;

        // slow mode = right trigger
        if (g1.right_trigger == 1) {
            multiplier = 0.4;
        }
        // fast mode = left trigger
        if (g1.left_trigger == 1) {
            multiplier = 1;
        }

        double yMult = multiplier;
        double xMult = multiplier;
        double turnMult = multiplier;

        if (y < 0) yMult *= -1;
        if (x < 0) xMult *= -1;
        if (turn < 0) turnMult *= -1;

        y = Math.pow(y, power) * yMult;
        x = Math.pow(x, power) * xMult;
        turn = Math.pow(turn, power) * turnMult;

        drive.setWeightedDrivePower(new Pose2d(y, x, turn));

        // right bumper -> close claw
        if (gamepad2.right_bumper) {
            Constants.setClaw(Constants.ClawPosition.CLOSED);
        }
        // left bumper -> close claw
        if (gamepad2.left_bumper) {
            Constants.setClaw(Constants.ClawPosition.OPEN);
        }

        // as you hold down right trigger, the extension will gradually go outward
        // when the trigger is 0, the extension will be at Constants.extendInPos
        // when the trigger is 1, the extension will be at Constants.extendOutPos
        // in between, the extension will be set proportionally
        extend.setPosition(gamepad2.right_trigger * (Constants.extendOutPos - Constants.extendInPos) + Constants.extendInPos);

        telemetry.addData("Left lift position", liftL.getCurrentPosition());
        telemetry.addData("Right lift position", liftR.getCurrentPosition());
        telemetry.addData("Turret position", turretR.getCurrentPosition());
        telemetry.update();
    }
}
