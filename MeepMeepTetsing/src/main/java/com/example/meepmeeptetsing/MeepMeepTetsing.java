package com.example.meepmeeptetsing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTetsing {
    private static final double TILE = 70.0 / 3;

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meep = new MeepMeep(800);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meep)
                .setConstraints(50, 50, Math.PI, Math.PI, 4850.573150389501 * 94.875 / 32064)
                .build();
        bot.runAction(
                bot.getDrive().actionBuilder(new Pose2d(-1.5 * TILE + 2, -2.5 * TILE, Math.toRadians(90)))
                        .splineTo(new Vector2d(-2 * TILE - 1, -1.5 * TILE), Math.toRadians(90))
                        .splineTo(new Vector2d(-2 * TILE - 1, -1 * TILE - 7), Math.toRadians(90))
                        .endTrajectory()
                        .setReversed(true)
                        .splineTo(new Vector2d(-2.5 * TILE, -2.5 * TILE), Math.toRadians(225))
                        .setReversed(false)
                        .splineTo(new Vector2d(-2.5 * TILE + 3, -2.5 * TILE + 3), Math.toRadians(45))
                        .endTrajectory()
                        .splineTo(new Vector2d(-2.5 * TILE, -1 * TILE - 7), Math.toRadians(90))
                        .setReversed(true)
                        .splineTo(new Vector2d(-2.5 * TILE, -2.5 * TILE), Math.toRadians(225))
                        .setReversed(false)
                        .splineTo(new Vector2d(-2.5 * TILE + 3, -2.5 * TILE + 3), Math.toRadians(45))
                        .build());
        meep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .addEntity(bot)
                .start();
    }
}