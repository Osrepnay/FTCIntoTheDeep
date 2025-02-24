package com.example.meepmeeptetsing;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.UnaryOperator;
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
                .setConstraints(70, 50, Math.PI, Math.PI * 1.5, 3768.610311809861 * 165.875 / 56205)
                .build();
        Action sample;
        {
            Pose2d basket = new Pose2d(-2.37 * TILE, -2.37 * TILE, Math.toRadians(45));
            Pose2d first = new Pose2d(-2.5 * TILE + 2.5, -1 * TILE - 31.5, Math.toRadians(72.7));
            Pose2d firstPushed = forward(first, 4);
            Pose2d second = new Pose2d(-2.5 * TILE + 0.5, -1 * TILE - 32, Math.toRadians(90));
            Pose2d secondPushed = forward(second, 4);
            Pose2d third = new Pose2d(-2.5 * TILE + 6.5, -2.5 * TILE + 6.5, Math.toRadians(119.7));
            Pose2d thirdPushed = forward(third, 4);
            Pose2d pickup = new Pose2d(-0.6 * TILE, -0.5 * TILE - 3, Math.toRadians(0));
            VelConstraint pushConstraint = new TranslationalVelConstraint(5);
            VelConstraint extendoConstraint = new AngularVelConstraint(Math.PI * 0.4);
            sample = bot.getDrive().actionBuilder(new Pose2d(-1.5 * TILE + 2.5, -2.5 * TILE - 5, Math.toRadians(0)))
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
        }

        Action specimen;
        {
            Pose2d chamber = new Pose2d(0.5 * TILE - 2, -1.5 * TILE - 1.1, Math.toRadians(-90));
            Vector2d adjust = new Vector2d(0, 0.15);
            Pose2d[] chambers = Stream.iterate(chamber, p -> new Pose2d(p.position.plus(adjust), p.heading))
                    .limit(5)
                    .toArray(Pose2d[]::new);
            Pose2d[] chamberSpin = Arrays.stream(chambers)
                    .map(p -> p.plus(new Twist2d(new Vector2d(10, 0), Math.toRadians(140))))
                    .toArray(Pose2d[]::new);
            Pose2d[] chamberShove = Arrays.stream(chambers)
                    .map(p -> p.plus(new Twist2d(new Vector2d(0, -3), Math.toRadians(0))))
                    .toArray(Pose2d[]::new);
            Pose2d pickup = new Pose2d(1.5 * TILE + 3, -2.5 * TILE + 2, Math.toRadians(89.99));
            Pose2d lineup = forward(pickup, 3);
            Pose2d moveFirst = new Pose2d(0.5 * TILE + 4.25 + 10, -2.5 * TILE - 3.5 + 19.5, Math.toRadians(33));
            Pose2d firstPushed = forward(moveFirst, 4.5);
            Pose2d moveSecond = new Pose2d(0.5 * TILE + 4.25 + 21.4, -2.5 * TILE - 3.5 + 19.5, Math.toRadians(35));
            Pose2d secondPushed = forward(moveSecond, 4);
            Pose2d moveThird = new Pose2d(0.5 * TILE + 4.25 + 29.1, -2.5 * TILE - 3.5 + 20.9, Math.toRadians(31));
            Pose2d thirdPushed = forward(moveThird, 4);
            Pose2d thirdDump = new Pose2d(0.5 * TILE + 4.25 + 22.2, -2.5 * TILE - 3.5 + 22, Math.toRadians(-51));
            VelConstraint lineupSlowdown = new AngularVelConstraint(Math.PI * 0.8);
            AccelConstraint lineupSlowdownAccel = new ProfileAccelConstraint(-25, 40);
            VelConstraint firstPushConstraint = new TranslationalVelConstraint(5);
            VelConstraint pushConstraint = new TranslationalVelConstraint(5);
            TurnConstraints raah = new TurnConstraints(
                    Math.PI * 1.5,
                    -Math.PI * 8,
                    Math.PI * 8
            );
            specimen = bot.getDrive().actionBuilder(new Pose2d(0.5 * TILE + 4.25, -2.5 * TILE - 3.5,
                            Math.toRadians(-90)))
                    .setReversed(true).splineTo(chambers[0].position, flip(chambers[0].heading))

                    // score second
                    .endTrajectory()
                    .strafeToSplineHeading(chamberSpin[1].position, chamberSpin[1].heading)
                    .splineToSplineHeading(lineup, flip(lineup.heading))
                    .strafeTo(pickup.position)
                    .strafeToSplineHeading(chambers[1].position, chambers[1].heading)
                    .endTrajectory()
                    .strafeToConstantHeading(chamberShove[1].position)

                    .endTrajectory().splineTo(moveFirst.position, moveFirst.heading, lineupSlowdown,
                            lineupSlowdownAccel)
                    .splineTo(firstPushed.position, firstPushed.heading, firstPushConstraint)
                    .turn(Math.toRadians(-85), raah)

                    .endTrajectory().splineToSplineHeading(
                            moveSecond,
                            moveSecond.heading.plus(Math.toRadians(-30)),
                            lineupSlowdown,
                            lineupSlowdownAccel)
                    .splineToConstantHeading(secondPushed.position, secondPushed.heading, pushConstraint)
                    .turn(Math.toRadians(-85), raah)

                    .endTrajectory().strafeToLinearHeading(moveThird.position, moveThird.heading, lineupSlowdown)
                    .strafeToConstantHeading(thirdPushed.position, pushConstraint)
                    .endTrajectory().strafeToSplineHeading(thirdDump.position, thirdDump.heading, lineupSlowdown)

                    .endTrajectory().splineToSplineHeading(lineup, flip(lineup.heading))
                    .strafeTo(pickup.position)
                    .strafeToSplineHeading(chambers[2].position, chambers[2].heading)

                    .strafeToSplineHeading(chamberSpin[2].position, chamberSpin[2].heading)
                    .splineToSplineHeading(lineup, flip(lineup.heading))
                    .strafeTo(pickup.position)
                    .strafeToSplineHeading(chambers[3].position, chambers[3].heading)
                    .build();
        }

        bot.runAction(specimen);
        meep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .addEntity(bot)
                .start();
    }
}