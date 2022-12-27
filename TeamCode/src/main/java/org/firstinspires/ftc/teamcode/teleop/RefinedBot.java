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

@TeleOp(name = "Refined Bot", group = "TestBot")
public class RefinedBot extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL = null;
    DcMotor liftR = null;
    DcMotor liftT = null;

    Servo claw = null;
    Servo extend = null;
    Servo tilt = null;

    DcMotor turret = null;

    Gamepad g1 = gamepad1;
    Gamepad g2 = gamepad2;

    double extensionPos = Constants.extendInPos;
    double extensionRange = Constants.extendOutPos - Constants.extendInPos;

    double tiltPos = Constants.tiltDownPosition;
    long timerStart = System.currentTimeMillis();

    boolean isHolding = false;
    boolean isOpen = false;
    boolean isMoving = false;

    int holdPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        g1 = gamepad1; //Defining gamepads Aarav Mehta
        g2 = gamepad2;

        Constants.initHardware(hardwareMap);

        liftL = Constants.liftL;
        liftR = Constants.liftR;
        liftT = Constants.liftT;

        claw = Constants.claw;
        extend = Constants.extend;
        tilt = Constants.tilt;

        turret = Constants.turret;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Constants.setClaw(Constants.ClawPosition.OPEN);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            teleopCode();
            teleopAuto();
        }
    }

    public void liftAuto(){
        // when you hold y, the lift will move upward until it hits the target position (liftTargetHigh)
        if ((g2.dpad_up || g2.y) && Math.abs(liftR.getCurrentPosition() - Constants.liftTargetHigh) >= Constants.liftError) {
            Constants.setLift(Constants.liftTargetHigh, Constants.liftPower); // you can see this method in Constants
        }
        // when you hold a, the lift will move downward until it gets down to 0
        if ((g2.dpad_down || g2.a) && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError) {
            Constants.setLift(0, Constants.liftPower);
        }
        //When you hold d, the lift will stay up and will hold Aarav Mehta

        // if neither a nor y are pressed, the right joystick will be controlling lift
        if (g2.dpad_left || Math.abs(g2.left_stick_x) > 0.2){
            if (!isHolding){
                isHolding = true;
                holdPos = liftR.getCurrentPosition();
            }
            Constants.setLift(holdPos, Constants.liftPower);
        }
        // if neither a nor y are pressed, the right joystick will be controlling lift
        if (!g2.dpad_down && !g2.dpad_up && !g2.a && !g2.y && !g2.dpad_left && Math.abs(g2.left_stick_x) <= 0.2) {
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // squared input
            double liftInput = square(gamepad2.left_stick_y) * Constants.liftPower;

            isHolding = false;
            holdPos = 0;

            // opposite powers
            if (liftR.getCurrentPosition() > Constants.liftLimit || gamepad2.left_stick_y < 0) {
                liftL.setPower(-liftInput);
                liftR.setPower(liftInput);
                liftT.setPower(-liftInput);
            }
        }
    }

    public void turretAuto(){
        //Dpad right -> turret goes to 90 degrees (right)
        if (g2.b && Math.abs(turret.getCurrentPosition() - Constants.turretTarget90) >= Constants.turretError) {
            Constants.setTurret(90, false, Constants.turretPower); // look at this method in Constants
        }
        //Dpad left -> turret goes to -90 degrees (left)
        else if (g2.x && Math.abs(turret.getCurrentPosition() - Constants.turretTargetNeg90) >= Constants.turretError) {
            Constants.setTurret(-90, false, Constants.turretPower);
        }
        //Dpad down -> turret goes to 180 degrees (backward)
        else if (g2.y && Math.abs(turret.getCurrentPosition() - Constants.turretTarget180) >= Constants.turretError) {
            Constants.setTurret(180, false, Constants.turretPower);
        }
        //Dpad up -> turret goes to 0 degrees (forward)
        else if (g2.a && Math.abs(turret.getCurrentPosition()) >= Constants.turretError) {
            Constants.setTurret(0, false, Constants.turretPower);
        }
        //If no dpads are pressed, left joystick will control turret
        else if (!g2.x && !g2.a && !g2.y && !g2.b) {
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // squared input, 0.5 speed
//            double turretInput = -Math.pow(gamepad2.right_stick_x, 2) * Constants.turretPower;
            double turretMultiplier = 0.7;

            double triggerDiff = g2.right_trigger - g2.left_trigger;
            double turretInput = -square(triggerDiff) * turretMultiplier;

            turret.setPower(turretInput);
        }
    }

    public void teleopAuto() {
        liftAuto();
        turretAuto();

        // press dpad right -> resets all encoders
        if (g2.dpad_right) {
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void armCode(){
        extensionPos += square(g2.right_stick_y) * Constants.extendSensitivity;

        if (gamepad2.right_stick_y == -1)
            extensionPos = Constants.extendInPos;

        if (extensionPos < Constants.extendOutPos)
            extensionPos = Constants.extendOutPos;
        if (extensionPos > Constants.extendInPos)
            extensionPos = Constants.extendInPos;

        if (g2.y || g2.a || turret.getPower() > 0.4)
            extensionPos = Constants.extendInPos;


        tiltPos += square(g2.right_stick_x) * Constants.tiltSensitivity;

        if (tiltPos > Constants.tiltUpPosition)
            tiltPos = Constants.tiltUpPosition;
        if (tiltPos < Constants.tiltDownPosition)
            tiltPos = Constants.tiltDownPosition;

        if (g2.y || g2.dpad_up || g2.left_stick_y > 0)
            tiltPos = Constants.tiltDropPosition;
        if (g2.a || g2.dpad_down || g2.left_stick_y < -0.7)
            tiltPos = Constants.tiltDownPosition;


        if (gamepad2.left_bumper) {
            extensionPos = Constants.extendOutPos;
            tiltPos = Constants.tiltDropPosition;
            if (!isMoving) {
                timerStart = System.currentTimeMillis();
                isMoving = true;
            }
        }
        if (isMoving && System.currentTimeMillis() - timerStart >= 300) {
            Constants.setClaw(Constants.ClawPosition.OPEN);
            tiltPos = Constants.tiltDownPosition;
            extensionPos = Constants.extendInPos;
            isMoving = false;
        }

        if (gamepad2.right_bumper) {
            if (isOpen)
                Constants.setClaw(Constants.ClawPosition.CLOSED);
            else
                Constants.setClaw(Constants.ClawPosition.OPEN);
            isOpen = !isOpen;
        }

        extend.setPosition(extensionPos);
        tilt.setPosition(tiltPos);

    }

    public void driveCode(){
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // only important numbers are here - default movement speed
        double multiplier = 0.75;
        // curve to apply (squared rn)
        double power = 2.0;

        // slow mode = right trigger
        if (g1.right_bumper) {
            multiplier = 0.3;
        }
        // fast mode = left trigger
        if (g1.left_bumper) {
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
    }

    public void telemetryCode(){
        telemetry.addData("Turret position", turret.getCurrentPosition());
        telemetry.addData("Lift position", liftR.getCurrentPosition());

        telemetry.addData("Extension position", extend.getPosition());
        telemetry.addData("Tilt position", tilt.getPosition());

        telemetry.update();
    }

    public void teleopCode() {
        driveCode();
        armCode();
        telemetryCode();
    }

    public double square(double input){
        double result = Math.pow(input,2);
        if (input < 0) result *= -1;
        return result;
    }
}
