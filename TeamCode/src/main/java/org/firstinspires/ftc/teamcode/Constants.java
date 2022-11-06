package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Constants {
    public static DcMotor liftL;
    public static DcMotor liftR;
    public static DcMotor turretR;

    public static Servo clawL;
    public static Servo clawR;

    public enum Color {
        RED,
        BLUE
    }

    public enum ClawPosition {
        OPEN,
        CLOSED
    }

    public static Color alliance;

    public static double waitTimeDrop = 0.75;
    public static double waitTimePickup = 0.75;

    public static int liftTargetHigh = 100;
    public static int liftError = 20;
    public static int turretError = 20;

    public static int turretTarget90 = 50;
    public static int turretTarget180 = 100;
    public static int turretTargetNeg90 = -50;

    public static double turretPower = 0.3;
    public static double liftPower = 0.5;

    public static double extendOutPos = 1.0;
    public static double extendInPos = 0.3;

    public static Pose2d startPoseL = new Pose2d(-36, -63, Math.toRadians(180));
    public static Pose2d startPoseR = new Pose2d(36, -63, Math.toRadians(180));
    public static Pose2d pickupL = new Pose2d(-61, -12, Math.toRadians(180));
    public static Pose2d mainDropL = new Pose2d(-24, -9, Math.toRadians(180));
    public static Pose2d preCycleL = new Pose2d(-15, -12, Math.toRadians(180));
    public static Pose2d firstAdjustmentL = new Pose2d(-12, -57, Math.toRadians(180));
    public static Pose2d firstDropL = new Pose2d(-9, -24, Math.toRadians(180));

    public static Vector2d parkMiddleL = new Vector2d(-60, -12);

    public static MarkerCallback prepareArmB = () -> {
        setClaw(ClawPosition.CLOSED);
        setLift(liftTargetHigh, liftPower);
        setTurret(180, false, turretPower);
    };

    public static MarkerCallback resetArm = () -> {
        setClaw(ClawPosition.OPEN);
        setLift(0,liftPower);
        setTurret(0,false,turretPower);
    };

    public static void setLift(int value, double power) {
        if (value < liftL.getCurrentPosition())
            power *= -1;

        liftL.setTargetPosition(value);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(power);

        liftR.setTargetPosition(-value);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setPower(-power);
    }

    public static void setTurret(int value, boolean isExact, double power) {
        int target = 0;
        if (isExact) {
            target = value;
        } else if (value == 90) {
            target = turretTarget90;
        } else if (value == 180) {
            target = turretTarget180;
        } else if (value == -90) {
            target = turretTargetNeg90;
        } else if (value == 0){
            target = 0;
        }
        if (target < turretR.getCurrentPosition())
            power *= -1;
        turretR.setTargetPosition((int) target);
        turretR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretR.setPower(power);
    }

    public static void setClaw(ClawPosition position) {
        if (position == ClawPosition.CLOSED) {
            clawL.setPosition(1);
            clawR.setPosition(0);
        } else {
            clawL.setPosition(.4);
            clawR.setPosition(.6);
        }
    }

    public static void initHardware(HardwareMap hardwareMap) {
        liftL = hardwareMap.dcMotor.get("lift_right");
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR = hardwareMap.dcMotor.get("lift_left");
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretR = hardwareMap.dcMotor.get("turret_right");
        turretR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawL = hardwareMap.servo.get("left_claw");
        clawR = hardwareMap.servo.get("right_claw");

        setClaw(ClawPosition.CLOSED);
    }
}
