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
    long timer2 = System.currentTimeMillis();

    boolean isHolding = false;
    boolean isOpen = true;
    boolean isMoving = false;
    boolean hasPressed = false;

    int holdPos = 0;

    double driveMultiplier = 0.75;

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
        if ((g2.dpad_up) && Math.abs(liftR.getCurrentPosition() - Constants.liftTargetHigh) >= Constants.liftError) {
            Constants.setLift(Constants.liftTargetHigh, Constants.liftPower);
        }
        if ((g2.dpad_left) && Math.abs(liftR.getCurrentPosition() - Constants.liftTargetMid) >= Constants.liftError) {
            Constants.setLift(Constants.liftTargetMid, Constants.liftPower);
        }
        if ((g2.dpad_right) && Math.abs(liftR.getCurrentPosition() - Constants.liftTargetLow) >= Constants.liftError) {
            Constants.setLift(Constants.liftTargetLow, Constants.liftPower);
        }
        if ((g2.dpad_down) && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError) {
            Constants.setLift(0, Constants.liftPower);
        }

        if (Math.abs(g2.left_stick_x) > 0.2){
            if (!isHolding){
                isHolding = true;
                holdPos = liftR.getCurrentPosition();
            }
            Constants.setLift(holdPos, Constants.liftPower);
        }
        if (!g2.dpad_down && !g2.dpad_up && !g2.dpad_right && !g2.dpad_left && Math.abs(g2.left_stick_x) <= 0.2) {
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double liftInput = square(gamepad2.left_stick_y) * Constants.liftPower;

            isHolding = false;
            holdPos = 0;

            if (liftR.getCurrentPosition() > Constants.liftLimit || gamepad2.left_stick_y < 0) {
                liftL.setPower(-liftInput);
                liftR.setPower(liftInput);
                liftT.setPower(-liftInput);
            }
        }
    }

    public void turretAuto(){
        if (g2.dpad_down && Math.abs(turret.getCurrentPosition()) >= Constants.turretError) {
            Constants.setTurret(0, false, 0.6);
        }
        else if (!g2.dpad_down) {
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double turretMultiplier = 0.5;

            double triggerDiff = g2.right_trigger - g2.left_trigger;
            double turretInput = -square(triggerDiff) * turretMultiplier;

            turret.setPower(turretInput);
        }
    }

    public void teleopAuto() {
        liftAuto();
        turretAuto();

        if (g2.b) {
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
        extensionPos += -(g2.right_stick_x) * Constants.extendSensitivity;

//        if (gamepad2.right_stick_y == -1)
//            extensionPos = Constants.extendInPos;

        if (extensionPos < Constants.extendOutPos)
            extensionPos = Constants.extendOutPos;
        if (extensionPos > Constants.extendInPos)
            extensionPos = Constants.extendInPos;

        if (g2.right_stick_button)
            extensionPos = Constants.extendInPos;

        if (g2.dpad_up || g2.dpad_down || turret.getPower() > 0.4)
            extensionPos = Constants.extendInPos - 0.2;


        tiltPos += -(g2.right_stick_y) * Constants.tiltSensitivity;

        if (tiltPos > Constants.tiltUpPosition)
            tiltPos = Constants.tiltUpPosition;

        if (g2.dpad_up)
            tiltPos = Constants.tiltDropPosition;
        if (g2.dpad_down)
            tiltPos = Constants.tiltDownPosition;

        if (g2.y)
            tiltPos = Constants.tiltUpPosition;
        if (g2.x)
            tiltPos = Constants.tiltDropPosition;
        if (g2.a)
            tiltPos = Constants.tiltDownPosition;


        if (gamepad2.left_bumper) {
            tiltPos = Constants.tiltDownPosition;
            if (!isMoving) {
                timerStart = System.currentTimeMillis();
                isMoving = true;
            }
        }


        if (isMoving && System.currentTimeMillis() - timerStart >= 200) {
            Constants.setClaw(Constants.ClawPosition.OPEN);
            extensionPos = Constants.extendInPos;
            isMoving = false;
            isOpen = true;
        }

        if (gamepad2.right_bumper) {
            if (!hasPressed) {
                timer2 = System.currentTimeMillis();
                hasPressed = true;

                if (isOpen)
                    Constants.setClaw(Constants.ClawPosition.CLOSED);
                else
                    Constants.setClaw(Constants.ClawPosition.OPEN);
                isOpen = !isOpen;
            }
        }
        if (System.currentTimeMillis() - timer2 >= 150){
            hasPressed = false;
        }

        extend.setPosition(extensionPos);
        tilt.setPosition(tiltPos);

    }

    public void driveCode(){
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        double power = 2.0;

        if (liftR.getCurrentPosition() < -2000)
            driveMultiplier = 0.4;
        else
            driveMultiplier = 0.75;

        if (g1.right_bumper) {
            driveMultiplier = 0.3;
        }
        if (g1.left_bumper) {
            driveMultiplier = 1;
        }

        double yMult = driveMultiplier;
        double xMult = driveMultiplier;
        double turnMult = driveMultiplier;

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
