package com.example.meepmeeptetsing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.ArrayList;
import java.util.List;
import java.util.function.UnaryOperator;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class MeepMeepTetsing {
    static class Chainer {
        TrajectoryActionBuilder builder;
        List<Action> actions = new ArrayList<>();

        public Chainer(TrajectoryActionBuilder tab) {
            builder = tab;
        }

        public Chainer add(UnaryOperator<TrajectoryActionBuilder> un) {
            builder = un.apply(builder);
            actions.add(builder.build());
            return this;
        }

        public Action build() {
            return new SequentialAction(actions.toArray(new Action[0]));
        }
    }

    private static Pose2d forward(Pose2d pose, double dist) {
        double dx = dist * Math.cos(pose.heading.toDouble());
        double dy = dist * Math.sin(pose.heading.toDouble());
        return new Pose2d(new Vector2d(pose.position.x + dx, pose.position.y + dy), pose.heading);
    }

    private static final double TILE = 70.0 / 3;

    private static Rotation2d flip(Rotation2d ang) {
        return ang.plus(Math.toRadians(180));
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meep = new MeepMeep(800);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meep)
                .setConstraints(50, 50, Math.PI * 0.8, Math.PI, 3768.610311809861 * 165.875 / 56205)
                .build();
        Pose2d basket = new Pose2d(-2.37 * TILE, -2.37 * TILE, Math.toRadians(45));
        Pose2d first = new Pose2d(-2.5 * TILE + 2.5, -1 * TILE - 31.5, Math.toRadians(72.7));
        Pose2d firstPushed = forward(first, 4);
        Pose2d second = new Pose2d(-2.5 * TILE + 2, -1 * TILE - 32, Math.toRadians(90));
        Pose2d secondPushed = forward(second, 4);
        Pose2d third = new Pose2d(-2.5 * TILE + 6.5, -2.5 * TILE + 6.5, Math.toRadians(119.7));
        Pose2d thirdPushed = forward(third, 4);
        VelConstraint pushConstraint = new TranslationalVelConstraint(5);
        VelConstraint extendoConstraint = new AngularVelConstraint(Math.PI * 0.4);
        Action sample = bot.getDrive().actionBuilder(new Pose2d(-1.5 * TILE + 2.5, -2.5 * TILE - 5, Math.toRadians(0)))
                .strafeToLinearHeading(basket.position, basket.heading)

                .endTrajectory().strafeToSplineHeading(first.position, first.heading, extendoConstraint)
                .endTrajectory().splineTo(firstPushed.position, firstPushed.heading, pushConstraint)
                .endTrajectory().strafeToSplineHeading(basket.position, basket.heading)

                .endTrajectory().strafeToSplineHeading(second.position, second.heading, extendoConstraint)
                .endTrajectory().splineTo(secondPushed.position, secondPushed.heading, pushConstraint)
                .endTrajectory().strafeToSplineHeading(basket.position, basket.heading)

                .endTrajectory().strafeToSplineHeading(third.position, third.heading, extendoConstraint)
                .endTrajectory().splineTo(thirdPushed.position, thirdPushed.heading, pushConstraint)
                .setReversed(true).splineTo(basket.position, flip(basket.heading))

                .build();

        Pose2d chamber = new Pose2d(0.5 * TILE - 5, -1.5 * TILE + 2, Math.toRadians(-90));
        Vector2d adjust = new Vector2d(-2.5, 0);
        Pose2d[] chambers = Stream.iterate(chamber, p -> new Pose2d(p.position.plus(adjust), p.heading))
                .limit(5)
                .toArray(Pose2d[]::new);
        Pose2d pickup = new Pose2d(1 * TILE - 1, -2.5 * TILE + 1, Math.toRadians(-15));
        Action specimen = bot.getDrive().actionBuilder(new Pose2d(0.5 * TILE + 3, -2.5 * TILE - 5.6, Math.toRadians(-90)))
                .setReversed(true).splineTo(chambers[0].position, flip(chambers[0].heading))
                .build();

        bot.runAction(specimen);
        meep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .addEntity(bot)
                .start();
    }
}