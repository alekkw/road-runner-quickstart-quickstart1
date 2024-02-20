package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 30, 3.13229, Math.toRadians(60), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, 61, Math.toRadians(-90)))

                                .lineToSplineHeading(new Pose2d(15, 61 - 18, Math.toRadians(180)))
                                .back(7)
                                .lineToSplineHeading(new Pose2d(15 + 20, 61- 18, Math.toRadians(0)))

//                                .splineTo(new Vector2d(15 + 30,61 - 30), Math.toRadians(0))
//
//                                .lineToSplineHeading(new Pose2d(15 + 20, 61 - 30, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(15 - 7, 61 - 1.5), Math.toRadians(180))
//                                .lineToLinearHeading(new Pose2d(15 + -48, 61 - 1.5, Math.toRadians(180)))
//                                .splineToSplineHeading(new Pose2d(15 - 64, 61 - 26, Math.toRadians(90)), Math.toRadians(180))
//                                .strafeLeft(6)
//
//                                .strafeRight(6)
//                                .splineToSplineHeading(new Pose2d(15 - 48,61 - 1.5, Math.toRadians(0)), Math.toRadians(0))
//                                .lineToLinearHeading(new Pose2d(15 - 7, 61 - 1.5, Math.toRadians(0)))
//                                .splineToSplineHeading(new Pose2d(15 + 30,61 - 30, Math.toRadians(-90)), Math.toRadians(0))
//
//                                .lineToSplineHeading(new Pose2d(15 + 20, 61 - 30, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(15 - 7, 61 - 1.5), Math.toRadians(180))
//                                .lineToLinearHeading(new Pose2d(15 + -48, 61 - 1.5, Math.toRadians(180)))
//                                .splineToSplineHeading(new Pose2d(15 - 64, 61 - 26, Math.toRadians(90)), Math.toRadians(180))
//                                .strafeLeft(6)
//
//                                .strafeRight(6)
//                                .splineToSplineHeading(new Pose2d(15 - 48,61 - 1.5, Math.toRadians(0)), Math.toRadians(0))
//                                .lineToLinearHeading(new Pose2d(15 - 7, 61 - 1.5, Math.toRadians(0)))
//                                .splineToSplineHeading(new Pose2d(15 + 30,61 - 30, Math.toRadians(-90)), Math.toRadians(0))
//
//                                .back(30)
//                                .strafeLeft(15)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}