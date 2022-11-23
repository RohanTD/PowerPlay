package com.example.meepmeeppowerplay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.util.*;
public class MeepMeepPowerPlay {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//        Pose2d startPoseL = new Pose2d(-36,-63,Math.toRadians(180));
//        Pose2d startPoseR = new Pose2d(36,-63,Math.toRadians(180));
//        Pose2d pickupL = new Pose2d(-61,-12,Math.toRadians(180));
//        Pose2d mainDropL = new Pose2d(-24,-9,Math.toRadians(180));

        Pose2d startPoseL = new Pose2d(-36, -63, Math.toRadians(180));
        Pose2d startPoseR = new Pose2d(36, -63, Math.toRadians(180));
        Pose2d pickupL = new Pose2d(-52, -12, Math.toRadians(180));
        Pose2d mainDropL = new Pose2d(-42, -12, Math.toRadians(180));
        Pose2d preCycleL = new Pose2d(-12, -12, Math.toRadians(180));
        Pose2d firstAdjustmentL = new Pose2d(-12, -60, Math.toRadians(180));
        Pose2d firstDropL = new Pose2d(-12, -45, Math.toRadians(180));

        double waitTime = 1.5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setStartPose(startPoseL)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseL)
                                .lineToLinearHeading(firstAdjustmentL)
                                .lineToLinearHeading(firstDropL)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(preCycleL)
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