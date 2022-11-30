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
    public static DcMotor liftT;
    public static DcMotor turretR;

    public static Servo clawL;
    public static Servo clawR;
    public static Servo extend = null;

    // Not used rn but jic
    public enum Color {
        RED,
        BLUE
    }

    // Instead of having to memorize the values this makes it easier
    public enum ClawPosition {
        OPEN,
        CLOSED
    }

    public static Color alliance;

    // Not used rn
    public static double waitTimeDrop = 0.75;
    public static double waitTimePickup = 0.75;

    public static int liftTargetHigh = -4000; // Encoder value for the lift in up position
    public static int liftError = 50; // Amount of error allowed for lift positions (sbf as is)
    public static int turretError = 20; // ^
    public static int liftLimit = -4500;

    public static int turretTarget90 = -740; // Encoder value for the turret at right 90 degree position
    public static int turretTarget180 = 1450; // Encoder value for the turret at back 180 degree position
    public static int turretTargetNeg90 = 750; // Encoder value for the turret at left 90 degree position

    public static double turretPower = 0.5; // Default turret power in auton and teleop automation
    public static double liftPower = 1; // Default lift power in auton and teleop automation

    public static double extendOutPos = 0.55; // Servo position on the extension when the extension is out
    public static double extendInPos = 1.0; // Servo position on the extension when the extenion is in
    public static double extendSensitivity = 0.03;

    public static Pose2d startPoseL = new Pose2d(-36, -63, Math.toRadians(180));
    public static Pose2d startPoseR = new Pose2d(36, -63, Math.toRadians(180));
    public static Pose2d pickupL = new Pose2d(-52, -12, Math.toRadians(180));
    public static Pose2d mainDropL = new Pose2d(-42, -12, Math.toRadians(180));
    public static Pose2d preCycleL = new Pose2d(-12, -12, Math.toRadians(180));
    public static Pose2d firstAdjustmentL = new Pose2d(-12, -60, Math.toRadians(180));
    public static Pose2d firstDropL = new Pose2d(-12, -45, Math.toRadians(180));


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
        // sets both lift motors to the value at the default power
        if (value > liftR.getCurrentPosition())
            power *= -1;

        liftL.setTargetPosition(-value);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(-power);


        liftR.setTargetPosition(value);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setPower(power);

        liftT.setTargetPosition(-value);
        liftT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftT.setPower(-power);
    }

    public static void setTurret(int value, boolean isExact, double power) {
        // isExact is true when we are giving an exact encoder value for the turret
        // otherwise, it will assume that the value is a degree number
        double powerT = -power;
        int target = 0;
        // set the target based on value and isExact
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
            powerT *= -1;
        turretR.setTargetPosition(target);
        turretR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretR.setPower(powerT);
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
        liftL = hardwareMap.dcMotor.get("lift_left");
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftR = hardwareMap.dcMotor.get("lift_right");
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftT = hardwareMap.dcMotor.get("lift_top");
        liftT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretR = hardwareMap.dcMotor.get("turret_right");
        turretR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawL = hardwareMap.servo.get("left_claw");
        clawR = hardwareMap.servo.get("right_claw");

        extend = hardwareMap.servo.get("extend");
    }
}
