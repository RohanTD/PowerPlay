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
    public static DcMotor turretR;

    public static Servo clawL;
    public static Servo clawR;
    public static Servo extend;

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

    public enum ExtendPosition{
        OUT,
        IN
    }

    public static Color alliance;
    public static Side side;

    public static double offsetTimeDrop = 1.2;
    public static double offsetTimePickup = 1.0;

    public static int redThresh = 150;
    public static int blueThresh = 150;
    public static int purpleThresh = 150;

    public static int regionX = 150;
    public static int regionY = 90;

    public static int liftTargetHigh = -2950; // Encoder value for the lift in up position
    public static int liftError = 50; // Amount of error allowed for lift positions (sbf as is)
    public static int turretError = 20; // ^
    public static int liftLimit = -3200;

    public static int turretTarget90 = -700; // Encoder value for the turret at right 90 degree position
    public static int turretTarget180 = 1400; // Encoder value for the turret at back 180 degree position
    public static int turretTargetNeg90 = 700; // Encoder value for the turret at left 90 degree position

    public static int coneStackHighPosition = -500;
    public static int coneStackInterval = 100;

    public static double turretPower = 1; // Default turret power in auton and teleop automation
    public static double liftPower = 1; // Default lift power in auton and teleop automation

    public static double extendOutPos = 0.55; // Servo position on the extension when the extension is out
    public static double extendRightPos = 0.7875;
    public static double extendBackPos = 0.765;
    public static double extendInPos = 1.0; // Servo position on the extension when the e1xtenion is in
    public static double extendSensitivity = 0.03;

    public static Pose2d startPoseL = new Pose2d(-33, -62, Math.toRadians(180));
    public static Pose2d startPoseR = new Pose2d(33, -63, Math.toRadians(180));
    public static Pose2d pickupL = new Pose2d(-51.65, -11.5, Math.toRadians(180));
    public static Pose2d mainDropL = new Pose2d(-22.5, -13, Math.toRadians(180));
    public static Pose2d LSecond = new Pose2d(pickupL.getX() - 0.5, -12, Math.toRadians(180));
    public static Pose2d preCycleL = new Pose2d(-12, -12, Math.toRadians(180));
    public static Pose2d firstAdjustmentL = new Pose2d(-12, -60, Math.toRadians(180));
    public static Pose2d firstDropL = new Pose2d(-12, -23, Math.toRadians(180));


    public static Vector2d parkMiddleL = new Vector2d(-60, -12);

    public static void setExtend(ExtendPosition position){
        if (position == ExtendPosition.IN){
            extend.setPosition(extendInPos);
        }
        else{
            extend.setPosition(extendOutPos);
        }
    }

    public static void dropAndReset(){
        Constants.setClaw(Constants.ClawPosition.OPEN);
        Constants.sleepTime(100);
        extend.setPosition(Constants.extendInPos);
        Constants.sleepTime(150);
        Constants.setLift(Constants.coneStackHighPosition,Constants.liftPower);
        Constants.setTurret(0,true,Constants.turretPower);

    }

    public static void prepareDrop(){
        sleepTime(200);
        Constants.setLift(liftTargetHigh,Constants.liftPower);
        Constants.sleepTime(200);
        extend.setPosition(Constants.extendInPos);
        Constants.sleepTime(500);

        int turretTarget = turretTarget90;
        if (side == Side.RIGHT)
            turretTarget = turretTargetNeg90;
        Constants.setTurret(turretTarget,true,Constants.turretPower);
    }

    public static void preparePickup(int currentCycleCounter){
        extend.setPosition(Constants.extendOutPos);
        Constants.setLift(Constants.coneStackHighPosition + (currentCycleCounter * Constants.coneStackInterval),Constants.liftPower);
    }

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
