package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(270));
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 20, Math.toRadians(360), Math.toRadians(60), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToLinearHeading(new Pose2d(12, 24, Math.toRadians(180)))
                                //.addTemporalMarker(() -> arm.goTo(detector::getLocationInt))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(1.74, 24))
                                //.addTemporalMarker(() -> intake.out(1))
                                .waitSeconds(0.5)
                                //.addTemporalMarker(() -> intake.stop())
                                .lineToLinearHeading(new Pose2d(12, 63, Math.toRadians(270)))
                                .strafeTo(new Vector2d(41, 63))
                                //.addTemporalMarker(() -> arm.goTo(0))
                                .lineToLinearHeading(new Pose2d(57, 52.5, Math.toRadians(0)))
                                //.addTemporalMarker(() -> intake.in(1))
                                .waitSeconds(0.5)
                                //.addTemporalMarker(() -> intake.stop())
                                //.addTemporalMarker(() -> arm.goTo(1))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(41, 63, Math.toRadians(270)))
                                .strafeTo(new Vector2d(12, 63))
                                .lineToLinearHeading(new Pose2d(12, 24, Math.toRadians(180)))
                                //.addTemporalMarker(() -> arm.goTo(detector::getLocationInt))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(1.74, 24))
                                //.addTemporalMarker(() -> intake.out(1))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(12, 63, Math.toRadians(270)))
                                .strafeTo(new Vector2d(41, 63))

                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}