package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

import java.util.stream.Stream;

@Autonomous
public class Bongo extends OpMode {
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
        drive = new MecanumDrive(hardwareMap, new Pose2d(0.5 * TILE + 3, -2.5 * TILE - 5.6, Math.toRadians(-90)));
        inp = new InputManager();
        inp.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> robot.getState().INTAKING && robot.extendo.hasSample(),
                () -> robot.extendo.setIntake(Extendo.INTAKE_OFF)
        ));
        Pose2d chamber = new Pose2d(0.5 * TILE - 5, -1.5 * TILE + 2, Math.toRadians(-90));
        Vector2d adjust = new Vector2d(-2.5, 0);
        Pose2d[] chambers = Stream.iterate(chamber, p -> new Pose2d(p.position.plus(adjust), p.heading))
                .limit(6)
                .toArray(Pose2d[]::new);
        Pose2d fromChamberPickup = new Pose2d(1 * TILE - 1, -2.5 * TILE + 1, Math.toRadians(-15));
        Pose2d fromChamberPickupPushed = forward(fromChamberPickup, 11);
        Pose2d firstSpec = new Pose2d(1 * TILE, -1.7 * TILE, Math.toRadians(30));
        Pose2d firstSpecPushed = new Pose2d(1 * TILE, -1.7 * TILE, Math.toRadians(30));
        taskRunner.sendTask(robot.init());
        taskRunner.sendTask(new TrajectoryTaskBuilder(drive.actionBuilder(drive.pose))
                .andThen(t -> t.setReversed(true)
                        .splineTo(chambers[0].position, chambers[0].heading.plus(Math.toRadians(180))))
                .withLast(robot.transitionTask(Robot.Input.BUTTON_X))
                .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))

                .andThen(t -> t.endTrajectory().fresh()
                        .setReversed(false)
                        .splineTo(fromChamberPickup.position, fromChamberPickup.heading))
                .withLast(Task.delay(1000)
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.BUTTON_X)))
                .andThen(t -> t.endTrajectory().fresh()
                        .splineTo(fromChamberPickupPushed.position, fromChamberPickupPushed.heading))
                .andThen(t -> t.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(chambers[1].position, chambers[1].heading.plus(Math.toRadians(180))))
                .withLast(robot.transitionTask(Robot.Input.RIGHT_BUMPER)
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.BUTTON_X)))
                .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))

                .andThen(t -> t.endTrajectory().fresh()
                        .setReversed(false)
                        .splineTo(fromChamberPickup.position, fromChamberPickup.heading))
                .withLast(Task.delay(1000)
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.BUTTON_X)))
                .andThen(t -> t.endTrajectory().fresh()
                        .splineTo(fromChamberPickupPushed.position, fromChamberPickupPushed.heading))
                .andThen(t -> t.endTrajectory().fresh()
                        .setReversed(true)
                        .splineTo(chambers[2].position, chambers[2].heading.plus(Math.toRadians(180))))
                .withLast(robot.transitionTask(Robot.Input.RIGHT_BUMPER)
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.BUTTON_X)))
                .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                .build()
        );
    }

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
