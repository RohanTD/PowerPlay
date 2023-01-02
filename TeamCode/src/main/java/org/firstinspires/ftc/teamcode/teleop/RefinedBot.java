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
    Servo retraction = null;
    Servo camera = null;

    DcMotor turret = null;

    Gamepad g1 = gamepad1;
    Gamepad g2 = gamepad2;

    double extensionPos = Constants.extendInPos;
    double extensionRange = Constants.extendOutPos - Constants.extendInPos;
    double cameraPos = Constants.cameraDownPos;

    double tiltPos = Constants.tiltDownPosition;
    long timerStart = System.currentTimeMillis();
    long timer2 = System.currentTimeMillis();

    boolean isHolding = false;
    boolean isOpen = true;
    boolean isMoving = false;
    boolean hasPressed = false;
    boolean hasPulled = false;
    boolean hasTilted = false;

    int resetStatus = 0;

    int holdPos = 0;

    double driveMultiplier = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        g1 = gamepad1; //Defining gs Aarav Mehta
        g2 = gamepad2;

        Constants.initHardware(hardwareMap);

        liftL = Constants.liftL;
        liftR = Constants.liftR;
        liftT = Constants.liftT;

        claw = Constants.claw;
        extend = Constants.extend;
        tilt = Constants.tilt;
        retraction = Constants.retraction;
        camera = Constants.camera;

        turret = Constants.turret;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        camera.setPosition(Constants.cameraDownPos);

        Constants.setClaw(Constants.ClawPosition.OPEN);
        retraction.setPosition(Constants.retractionUpPos);

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
        if ((g2.dpad_down) && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError && Math.abs(turret.getCurrentPosition()) >= Constants.turretError) {
            Constants.setLift(-600, Constants.liftPower);
            resetStatus = 1;
        }
        else if ((g2.dpad_down) && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError) {
            Constants.setLift(0, Constants.liftPower);
            resetStatus = 2;
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

            resetStatus = 0;
            double liftInput = square(g2.left_stick_y) * Constants.liftPower;

            isHolding = false;
            holdPos = 0;

            if (liftR.getCurrentPosition() > Constants.liftLimit || g2.left_stick_y < 0) {
                liftL.setPower(-liftInput);
                liftR.setPower(liftInput);
                liftT.setPower(-liftInput);
            }
        }
    }

    public void turretAuto(){
        double turretMultiplier = 0.5;
        if (g2.dpad_down && Math.abs(turret.getCurrentPosition()) >= Constants.liftError) {
            Constants.setTurret(0, false, turretMultiplier);
        }
        if (g2.right_trigger == 1 && g2.left_trigger == 1 && Math.abs(turret.getCurrentPosition() - Constants.turretTarget180) >= Constants.liftError){
            Constants.setTurret(180, false, turretMultiplier);
        }
        if (!g2.dpad_down && !(g2.right_trigger == 1 && g2.left_trigger == 1)) {
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        if (g2.right_stick_x > 0 && liftR.getCurrentPosition() >= -500) {
            extensionPos += -(g2.right_stick_x) * Constants.extendSensitivity;
        }
        else if (g2.right_stick_x > 0){
            extensionPos += -(g2.right_stick_x) * Constants.extendSensitivity * 5;
        }
        else {
            extensionPos += -(g2.right_stick_x) * Constants.extendSensitivity;
        }

//        if (g2.right_stick_x == 1)
//            extensionPos = Constants.extendOutPos;

        if (extensionPos < Constants.extendOutPos)
            extensionPos = Constants.extendOutPos;
        if (extensionPos > Constants.extendInPos)
            extensionPos = Constants.extendInPos;

        if (g2.right_stick_button)
            extensionPos = Constants.extendInPos;

        if (resetStatus == 1){
            extensionPos = Constants.extendInPos - 0.1;
            hasPulled = false;
        }
        if (resetStatus == 2){
            extensionPos = Constants.extendInPos - 0.3;
            hasPulled = false;
        }

        if ((g2.dpad_up || g2.dpad_left || g2.dpad_right) && !hasPulled) {
            extensionPos = Constants.extendInPos;
            hasPulled = true;
        }

//        if (Math.abs(turret.getPower()) == 0.5)
//            extensionPos = Constants.extendInPos;


        tiltPos += -(g2.right_stick_y) * Constants.tiltSensitivity;

        if (tiltPos > Constants.tiltUpPosition)
            tiltPos = Constants.tiltUpPosition;

        if ((g2.dpad_up || g2.dpad_left || g2.dpad_right) && !hasTilted) {
            tiltPos = Constants.tiltDropPosition;
            hasTilted = true;
        }
//        if (g2.dpad_down && Math.abs(turret.getCurrentPosition()) >= Constants.turretError) {
//            tiltPos = Constants.tiltUpPosition;
//            hasTilted = false;
//        }
        else if (g2.dpad_down) {
            tiltPos = Constants.tiltDownPosition;
            hasTilted = false;
        }

        if (g2.y)
            tiltPos = Constants.tiltUpPosition;
        if (g2.x)
            tiltPos = Constants.tiltDropPosition;
        if (g2.a) {
            tiltPos = Constants.tiltDownPosition;
            hasTilted = false;
        }


        if (g2.left_bumper) {
            tiltPos = Constants.tiltDownPosition;
            hasTilted = false;
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

        if (g2.right_bumper) {
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
        if (System.currentTimeMillis() - timer2 >= 250){
            hasPressed = false;
        }

        extend.setPosition(extensionPos);
        tilt.setPosition(tiltPos);

    }

    public void driveCode(){
        double y = -g1.left_stick_y;
        double x = -g1.left_stick_x;
        double turn = -g1.right_stick_x;

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

        if (g1.dpad_up)
            retraction.setPosition(Constants.retractionUpPos);
        if (g1.dpad_down)
            retraction.setPosition(Constants.retractionDownPos);

        if (g1.dpad_right)
            cameraPos += 0.01;
        if (g1.dpad_left)
            cameraPos -= 0.01;
        camera.setPosition(cameraPos);
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
