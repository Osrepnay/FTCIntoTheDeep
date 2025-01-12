package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@Autonomous
public class LargeMonkey extends OpMode {
    Robot robot;
    TaskRunner taskRunner;
    MecanumDrive drive;
    InputManager inp;

    private final double TILE = 70.0 / 3;

    private static Pose2d forward(Pose2d pose, double dist) {
        double dx = dist * Math.cos(pose.heading.toDouble());
        double dy = dist * Math.sin(pose.heading.toDouble());
        return new Pose2d(new Vector2d(pose.position.x + dx, pose.position.y + dy), pose.heading);
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        taskRunner = new TaskRunner();
        drive = new MecanumDrive(hardwareMap, new Pose2d(-1.5 * TILE + 2.5, -2.5 * TILE - 5, Math.toRadians(0)));
        inp = new InputManager();
        inp.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> robot.getState().INTAKING && robot.extendo.hasCorrectSample(),
                () -> robot.extendo.setIntake(Extendo.INTAKE_OFF)
        ));
        Pose2d basket = new Pose2d(-2.4 * TILE, -2.4 * TILE, Math.toRadians(45));
        Pose2d first = new Pose2d(-2 * TILE + 1, -2 * TILE - 3, Math.toRadians(90));
        Pose2d firstPushed = forward(first, 8);
        Pose2d second = new Pose2d(-2.5 * TILE + 2, -2 * TILE - 3, Math.toRadians(88));
        Pose2d secondPushed = forward(second, 8);
        Pose2d third = new Pose2d(-2.5 * TILE + 1, -2 * TILE - 2, Math.toRadians(107));
        Pose2d thirdPushed = forward(third, 6);
        taskRunner.sendTask(robot.init());
        taskRunner.sendTask(new TrajectoryTaskBuilder(drive.actionBuilder(drive.pose))
                .andThen(t -> t.strafeToLinearHeading(basket.position, basket.heading))
                .withLast(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))

                .andThen(t -> t.endTrajectory().fresh().strafeToLinearHeading(first.position, first.heading))
                .withLast(robot.transitionTask(Robot.Input.LEFT_BUMPER)
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER)))
                .andThen(t -> t.endTrajectory().fresh().strafeToConstantHeading(firstPushed.position))
                .andThen(t -> t.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(basket.position, basket.heading.plus(Math.toRadians(180))))
                .withLast(robot.transitionTask(Robot.Input.RIGHT_BUMPER)
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER)))
                .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))

                .andThen(t -> t.endTrajectory().fresh().strafeToLinearHeading(second.position, second.heading))
                .withLast(robot.transitionTask(Robot.Input.LEFT_BUMPER)
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER)))
                .andThen(t -> t.endTrajectory().fresh().strafeToConstantHeading(secondPushed.position))
                .andThen(t -> t.endTrajectory().fresh().strafeToLinearHeading(basket.position, basket.heading))
                .withLast(robot.transitionTask(Robot.Input.RIGHT_BUMPER)
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER)))
                .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))

                .andThen(t -> t.endTrajectory().fresh().strafeToLinearHeading(third.position, third.heading))
                .withLast(robot.transitionTask(Robot.Input.LEFT_BUMPER)
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER)))
                .andThen(t -> t.endTrajectory().fresh().strafeToConstantHeading(thirdPushed.position))
                .andThen(t -> t.endTrajectory().fresh().strafeToLinearHeading(basket.position, basket.heading))
                .withLast(robot.transitionTask(Robot.Input.RIGHT_BUMPER)
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER)))
                .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))

                .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                .build()
        );
    }

    boolean first = true;
    boolean stopped = false;

    @Override
    public void loop() {
        telemetry.addData("state", robot.getState());
        telemetry.addData("lift", robot.lift.liftSetpoint);

        taskRunner.update();
        // TODO?
        drive.updatePoseEstimate();
        robot.update();
    }
}
