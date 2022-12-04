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

@TeleOp(name = "Outreach Teleop", group = "TestBot")
public class OutreachTeleop extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL = null;
    DcMotor liftR = null;
    DcMotor liftT = null;

    Servo clawL = null;
    Servo clawR = null;
    Servo extend = null;

    DcMotor turretR = null;

    Gamepad g1 = gamepad1;
    Gamepad g2 = gamepad2;

    double extensionPos = Constants.extendInPos;
    double extensionRange = Constants.extendOutPos - Constants.extendInPos;
    boolean isHolding = false;

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

        clawL = Constants.clawL; //Left and right servos on claws Aarav Mehta
        clawR = Constants.clawR;

        extend = Constants.extend;

        turretR = Constants.turretR;

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
        if ((g2.dpad_up || g2.y) && Math.abs(liftR.getCurrentPosition() - Constants.liftTargetHigh) >= Constants.liftError) {
            Constants.setLift(Constants.liftTargetHigh, Constants.liftPower); // you can see this method in Constants
        }
        // when you hold a, the lift will move downward until it gets down to 0
        if ((g2.dpad_down || g2.a) && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError) {
            Constants.setLift(0, Constants.liftPower);
        }
        //When you hold d, the lift will stay up and will hold Aarav Mehta

        // if neither a nor y are pressed, the right joystick will be controlling lift
        if (!g2.dpad_down && !g2.dpad_up && !g2.a && !g2.y && !g2.dpad_left) {
//            if (g2.left_stick_y == 0 && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError){
//                if (!isHolding){
//                    isHolding = true;
//                    holdPos = liftR.getCurrentPosition();
//                }
//                Constants.setLift(holdPos, Constants.liftPower);
//            }
//            else {
                liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // squared input
                double liftInput = Math.pow(gamepad2.left_stick_y, 2);
                if (gamepad2.left_stick_y < 0) liftInput *= -1;

//                isHolding = false;
//                holdPos = 0;

                // opposite powers
                if (liftR.getCurrentPosition() > Constants.liftLimit || gamepad2.left_stick_y < 0) {
                    liftL.setPower(-liftInput);
                    liftR.setPower(liftInput);
                    liftT.setPower(-liftInput);
                }
//            }
        }

        //Dpad right -> turret goes to 90 degrees (right)
        if (g2.b && Math.abs(turretR.getCurrentPosition() - Constants.turretTarget90) >= Constants.turretError) {
            Constants.setTurret(90, false, Constants.turretPower); // look at this method in Constants
        }
        //Dpad left -> turret goes to -90 degrees (left)
        else if (g2.x && Math.abs(turretR.getCurrentPosition() - Constants.turretTargetNeg90) >= Constants.turretError) {
            Constants.setTurret(-90, false, Constants.turretPower);
        }
        //Dpad down -> turret goes to 180 degrees (backward)
        else if (g2.y && Math.abs(turretR.getCurrentPosition() - Constants.turretTarget180) >= Constants.turretError) {
            Constants.setTurret(180, false, Constants.turretPower);
        }
        //Dpad up -> turret goes to 0 degrees (forward)
        else if (g2.a && Math.abs(turretR.getCurrentPosition()) >= Constants.turretError) {
            Constants.setTurret(0, false, Constants.turretPower);
        }
        //If no dpads are pressed, left joystick will control turret
        else if (!g2.x && !g2.a && !g2.y && !g2.b) {
            turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // squared input, 0.5 speed
            double turretInput = -Math.pow(gamepad2.right_stick_x, 2) * Constants.turretPower;
            // left trigger = slow mode
//            if (g2.left_trigger == 1) turretInput *= 0.7;
            if (gamepad2.right_stick_x < 0) turretInput *= -1;

            turretR.setPower(turretInput);
        }

        // press b -> resets all encoders
        if (g2.dpad_right) {
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turretR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void teleopCode() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // only important numbers are here - default movement speed
        double multiplier = 0.4;
        // curve to apply (squared rn)
        double power = 1.0;

        double yMult = multiplier;
        double xMult = multiplier;
        double turnMult = multiplier;

        y = Math.pow(y, power) * yMult;
        x = Math.pow(x, power) * xMult;
        turn = Math.pow(turn, power) * turnMult;

        if (g2.b) {
            y = 0;
            x = 0;
            turn = 0;
        }

        drive.setWeightedDrivePower(new Pose2d(y, x, turn));

        // right bumper -> close claw
        if (gamepad1.right_bumper) {
            Constants.setClaw(Constants.ClawPosition.CLOSED);
        }
        // left bumper -> close claw
        if (gamepad1.left_bumper) {
            Constants.setClaw(Constants.ClawPosition.OPEN);
        }

        // as you hold down right trigger, the extension will gradually go outward
        // when the trigger is 0, the extension will be at Constants.extendInPos
        // when the trigger is 1, the extension will be at Constants.extendOutPos
        // in between, the extension will be set proportionally

//        double extensionValue = (Math.sqrt(gamepad2.right_trigger) + Math.sqrt(gamepad2.left_trigger)) / 2.0;
//        extend.setPosition((1 - extensionValue) * (Constants.extendInPos - Constants.extendOutPos) + Constants.extendOutPos);

        if (gamepad1.right_trigger <= 1)
            extensionPos += (extensionRange * gamepad1.right_trigger * Constants.extendSensitivity);
//        else if (gamepad2.right_trigger == 1)
//            extensionPos = Constants.extendOutPos;

        if (gamepad1.left_trigger < 1)
            extensionPos -= (extensionRange * gamepad1.left_trigger * Constants.extendSensitivity);
        else if (gamepad1.left_trigger == 1)
            extensionPos = Constants.extendInPos;

        if (extensionPos < Constants.extendOutPos)
            extensionPos = Constants.extendOutPos;
        if (extensionPos > Constants.extendInPos)
            extensionPos = Constants.extendInPos;

        if (g2.y || g2.a)
            extensionPos = Constants.extendInPos;

        extend.setPosition(extensionPos);


        telemetry.addData("Turret power", turretR.getPower());
        telemetry.addData("Turret position", turretR.getCurrentPosition());
        telemetry.addData("Turret target", turretR.getTargetPosition());

        telemetry.addData("Lift position", liftR.getCurrentPosition());
        telemetry.addData("Lift power", liftL.getPower());
        telemetry.addData("LiftT target", liftT.getTargetPosition());

        telemetry.addData("Extension position", extend.getPosition());

        telemetry.update();
    }
}
