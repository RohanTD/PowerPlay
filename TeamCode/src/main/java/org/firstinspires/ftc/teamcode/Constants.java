package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.slf4j.Marker;

public class Constants {
    public static DcMotor liftL;
    public static DcMotor liftR;
    public static DcMotor liftT;
    public static DcMotor turret;

    public static Servo claw;
    public static Servo extend;
    public static Servo tilt;
    public static Servo retraction;
    public static Servo camera;

    // Not used rn but jic
    public enum Color {
        RED,
        BLUE
    }

    public enum Side {
        RIGHT,
        LEFT
    }

    // Instead of having to memorize the values this makes it easier
    public enum ClawPosition {
        OPEN,
        CLOSED
    }



    public static Color alliance;
    public static Side side;

    public static double extendOffset = 0.7;
    public static double secondExtendOffset = 0.8;
    public static double preparePreloadOffset = 0.6;
    public static double finishLiftPreloadOffset = 1.3;

    public static int redThresh = 150;
    public static int blueThresh = 150;
    public static int purpleThresh = 150;

    public static int regionX = 140;
    public static int regionY = 90;

    public static int liftTargetHigh = -2290; // Encoder value for the lift in up position
    public static int liftTargetHighTeleop = -2350;
    public static int liftTargetMid = -1590;
    public static int liftTargetLow = -780;
    public static int liftError = 20; // Amount of error allowed for lift positions (sbf as is)
    public static int turretError = 43; // ^
    public static int liftLimit = -3200;

    public static int turretTarget90 = -700; // Encoder value for the turret at right 90 degree position
    public static int turretTarget180 = 1360; // Encoder value for the turret at back 180 degree position
    public static int turretTargetNeg90 = 700; // Encoder value for the turret at left 90 degree position
    public static int turretTargetAutonL = -1080;
    public static int turretTargetAutonR = 1000;
    public static int turretTargetTopRight = -400;
    public static int turretTargetTopLeft = 400;

    public static int coneStackHighPosition = -430;
    public static int coneStackInterval = 100;

    public static double turretPower = 0.7; // Default turret power in auton and teleop automation
    public static double liftPower = 1; // Default lift power in auton and teleop automation

    public static double extendOutPos = 0.47; // Servo position on the extension when the extension is out
    public static double extendRightPos = 0.7875;
    public static double extendBackPos = 0.765;
    public static double extendInPos = 0.94 ; // Servo position on the extension when the e1xtenion is in
    public static double extendSensitivity = 0.005;

    public static double tiltUpPosition = 1;
    public static double tiltDownPosition = 0.42;
    public static double tiltDropPosition = 0.65;
    public static double tiltSensitivity = 0.005;

    public static double cameraUpPos = .42;
    public static double cameraDownPos = .9;

    public static double retractionUpPos = 1;
    public static double retractionDownPos = 0;

    public static Pose2d startPoseL = new Pose2d(-33, -62, Math.toRadians(180));
    public static Pose2d pickupL = new Pose2d(-52.75, -11, Math.toRadians(180));
    public static Pose2d mainDropL = new Pose2d(-22.5, -13, Math.toRadians(180));
    public static Pose2d altDropL = new Pose2d(-36, -11.5,Math.toRadians(180));
    public static Pose2d pushOutL = new Pose2d(-36,-9,Math.toRadians(180));
    public static Pose2d cutAcrossL = new Pose2d(-36, -12, Math.toRadians(180));
    public static Pose2d preCycleL = new Pose2d(-12, -12, Math.toRadians(180));
    public static Pose2d firstAdjustmentL = new Pose2d(-12, -60, Math.toRadians(180));
    public static Pose2d firstDropL = new Pose2d(-12, -23, Math.toRadians(180));

    public static Pose2d startPoseR = new Pose2d(39, -63, Math.toRadians(180));
    public static Pose2d pickupR = new Pose2d(54.9, -12.5, Math.toRadians(0));
    public static Pose2d mainDropR = new Pose2d(22.5, -13, Math.toRadians(0));
    public static Pose2d turnAdjustmentR = new Pose2d(36,-11.75,Math.toRadians(0));
    public static Pose2d altDropR = new Pose2d(36, -11.5,Math.toRadians(0));
    public static Pose2d pushOutR = new Pose2d(36,-9,Math.toRadians(180));
    public static Pose2d cutAcrossR = new Pose2d(36, -12, Math.toRadians(180));
    public static Pose2d preCycleR = new Pose2d(12, -12, Math.toRadians(0));
    public static Pose2d firstAdjustmentR = new Pose2d(36, -63, Math.toRadians(180));
    public static Pose2d firstDropR = new Pose2d(12, -23, Math.toRadians(0));

    public static Vector2d parkLeftL = new Vector2d(-60, -12);
    public static Vector2d parkCenterL = new Vector2d(-36, -12);
    public static Vector2d parkRightL = new Vector2d(-12,-12);

    public static Vector2d parkLeftR = new Vector2d(60, -12);
    public static Vector2d parkCenterR = new Vector2d(36, -12);
    public static Vector2d parkRightR = new Vector2d(12,-12);

    public static void dropAndReset(){
        Constants.setClaw(Constants.ClawPosition.OPEN);
        Constants.sleepTime(100);
        extend.setPosition(Constants.extendInPos);
        Constants.sleepTime(150);
        Constants.setLift(Constants.coneStackHighPosition,Constants.liftPower);
        Constants.setTurret(0,true,Constants.turretPower);
    }

    public static void preloadDrop(){
        extend.setPosition(Constants.extendOutPos);
        tilt.setPosition(Constants.tiltDropPosition);
        Constants.sleepTime(400);
    }

    public static void tiltDrop(){
        extend.setPosition(Constants.extendOutPos);
        tilt.setPosition(Constants.tiltDownPosition);
        Constants.sleepTime(100);

        Constants.setClaw(Constants.ClawPosition.OPEN);
        extend.setPosition(Constants.extendInPos);
        Constants.sleepTime(300);
        Constants.setTurret(0,true,Constants.turretPower);
        Constants.setLift(Constants.coneStackHighPosition,Constants.liftPower);
        Constants.sleepTime(200);
    }

    public static void collect(){
        extend.setPosition(Constants.extendOutPos);
        Constants.setClaw(Constants.ClawPosition.CLOSED);
        Constants.sleepTime(300);

        Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
        Constants.sleepTime(200);
        extend.setPosition(Constants.extendInPos);
        tilt.setPosition(Constants.tiltDropPosition);
        Constants.sleepTime(200);
        Constants.setLift(Constants.liftTargetHigh,0);

        int turretTarget = turretTargetAutonL;
        if (side == Side.RIGHT)
            turretTarget = turretTargetAutonR;
        Constants.setTurret(turretTarget,true,Constants.turretPower);
        Constants.sleepTime(300);
        Constants.setLift(Constants.liftTargetHigh,Constants.liftPower);
    }

    public static MarkerCallback preparePreload = () -> {
        extend.setPosition(Constants.extendInPos);
        Constants.setLift(Constants.liftTargetHigh, 0);
        tilt.setPosition(Constants.tiltDropPosition);
        int turretTarget = turretTargetAutonL;
        if (side == Side.RIGHT)
            turretTarget = turretTargetAutonR;
        Constants.setTurret(turretTarget,true,0.5);
    };

    public static void sleepTime(long millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

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
        if (target < turret.getCurrentPosition())
            powerT *= -1;
        turret.setTargetPosition(target);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(powerT);
    }

    public static void setClaw(ClawPosition position) {
        if (position == ClawPosition.CLOSED) {
            claw.setPosition(1);
        } else {
            claw.setPosition(0.5);
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

        turret = hardwareMap.dcMotor.get("turret_right");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("claw");
        extend = hardwareMap.servo.get("extend");
        tilt = hardwareMap.servo.get("tilt");
        camera = hardwareMap.servo.get("camera");

        retraction = hardwareMap.servo.get("retraction");
    }
}
