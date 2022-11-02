package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Constants {
    public static DcMotor liftL;
    public static DcMotor liftR;
    public static DcMotor turretR;

    public static Servo clawL;
    public static Servo clawR;

    public static Pose2d startPoseL = new Pose2d(-36,-63,Math.toRadians(180));
    public static Pose2d startPoseR = new Pose2d(36,-63,Math.toRadians(180));
    public static Pose2d pickupL = new Pose2d(-61,-12,Math.toRadians(180));
    public static Pose2d mainDropL = new Pose2d(-24,-9,Math.toRadians(180));

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
    }
}
