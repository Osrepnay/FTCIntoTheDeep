package com.example.meepmeeptetsing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meep = new MeepMeep(800);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meep)
                .setConstraints(50, 50, Math.PI, Math.PI, 3768.610311809861 * 165.875 / 56205)
                .build();
        Pose2d basket = new Pose2d(-2.4 * TILE, -2.4 * TILE, Math.toRadians(45));
        Pose2d first = new Pose2d(-2 * TILE - 0.5, -2 * TILE - 5, Math.toRadians(90));
        Pose2d firstPushed = forward(first, 8);
        Pose2d second = new Pose2d(-2.5 * TILE + 2, -2 * TILE - 5, Math.toRadians(88));
        Pose2d secondPushed = forward(second, 8);
        Pose2d third = new Pose2d(-2.5 * TILE + -0.5, -2 * TILE - 2, Math.toRadians(109));
        Pose2d thirdPushed = forward(third, 6);
        Action sample = new Chainer(bot.getDrive().actionBuilder(new Pose2d(-1.5 * TILE + 2.5, -2.5 * TILE - 5, Math.toRadians(0))))
                .add(t -> t.strafeToLinearHeading(basket.position, basket.heading))
                .add(t -> t.endTrajectory().fresh().strafeToLinearHeading(first.position, first.heading))
                .add(t -> t.endTrajectory().fresh().strafeToConstantHeading(firstPushed.position))
                .add(t -> t.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(basket.position, basket.heading.plus(Math.toRadians(180))))
                .add(t -> t.endTrajectory().fresh().strafeToLinearHeading(second.position, second.heading))
                .add(t -> t.endTrajectory().fresh().strafeToConstantHeading(secondPushed.position))
                .add(t -> t.endTrajectory().fresh().strafeToLinearHeading(basket.position, basket.heading))
                .add(t -> t.endTrajectory().fresh().strafeToLinearHeading(third.position, third.heading))
                .add(t -> t.endTrajectory().fresh().strafeToConstantHeading(thirdPushed.position))
                .add(t -> t.endTrajectory().fresh().strafeToLinearHeading(basket.position, basket.heading))
                .build();

        Pose2d chamber = new Pose2d(0.5 * TILE - 5, -1.5 * TILE + 1.5, Math.toRadians(-90));
        Vector2d adjust = new Vector2d(-2.5, 0);
        Pose2d[] chambers = Stream.iterate(chamber, p -> new Pose2d(p.position.plus(adjust), p.heading))
                .limit(6)
                .toArray(Pose2d[]::new);
        Pose2d fromChamberPickup = new Pose2d(1 * TILE - 5, -2.5 * TILE - 3, Math.toRadians(0));
        Pose2d fromChamberPickupPushed = forward(fromChamberPickup, 11);
        Pose2d firstSpec = new Pose2d(1 * TILE, -1.7 * TILE, Math.toRadians(30));
        Pose2d firstSpecPushed = new Pose2d(1 * TILE, -1.7 * TILE, Math.toRadians(30));
        Action specimen = new Chainer(bot.getDrive().actionBuilder(new Pose2d(0.5 * TILE + 3, -2.5 * TILE - 5.6, Math.toRadians(-90))))
                .add(t -> t.setReversed(true).splineTo(chambers[0].position, chambers[0].heading.plus(Math.toRadians(180))))

                .add(t -> t.endTrajectory().fresh()
                        .setReversed(false)
                        .splineTo(fromChamberPickup.position, fromChamberPickup.heading))
                .add(t -> t.endTrajectory().fresh()
                        .splineTo(fromChamberPickupPushed.position, fromChamberPickupPushed.heading))
                .add(t -> t.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(chambers[1].position, chambers[1].heading.plus(Math.toRadians(180))))

                .add(t -> t.endTrajectory().fresh()
                        .setReversed(false)
                        .splineTo(fromChamberPickup.position, fromChamberPickup.heading))
                .add(t -> t.endTrajectory().fresh()
                        .splineTo(fromChamberPickupPushed.position, fromChamberPickupPushed.heading))
                .add(t -> t.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(chambers[2].position, chambers[2].heading.plus(Math.toRadians(180))))

                .add(t -> t.endTrajectory().fresh()
                        .setReversed(false)
                        .splineTo(firstSpec.position, firstSpec.heading))
                .add(t -> t.endTrajectory().fresh()
                        .splineTo(firstSpecPushed.position, firstSpecPushed.heading)
                        .endTrajectory()
                        .turn(Math.toRadians(-45)))
                .build();

        bot.runAction(specimen);
        meep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .addEntity(bot)
                .start();
    }
}