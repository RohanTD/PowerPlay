package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.firstinspires.ftc.teamcode.auton.Constants;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPoseL = Constants.startPose;
        Pose2d startPoseR = new Pose2d(36,-63,Math.toRadians(180));
        Pose2d pickupL = new Pose2d(-61,-12,Math.toRadians(180));
        Pose2d mainDropL = new Pose2d(-24,-9,Math.toRadians(180));

        double waitTime = 0.75;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setStartPose(startPoseL)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseL)
                                .lineToLinearHeading(new Pose2d(-12,-57,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-9,-24,Math.toRadians(180)))
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(new Pose2d(-15,-12,Math.toRadians(180)))
                                .lineToLinearHeading(pickupL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(mainDropL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(pickupL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(mainDropL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(pickupL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(mainDropL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(pickupL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(mainDropL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(pickupL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(mainDropL)
                                .waitSeconds(waitTime)
                                .lineTo(new Vector2d(-60,-12))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}