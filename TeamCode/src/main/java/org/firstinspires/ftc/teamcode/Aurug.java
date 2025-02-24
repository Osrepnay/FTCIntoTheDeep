package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TrajectoryTaskBuilder.actionToTask;
import static org.firstinspires.ftc.teamcode.TrajectoryTaskBuilder.taskToAction;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@Autonomous
public class Aurug extends OpMode {
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

    private static Rotation2d flip(Rotation2d ang) {
        return ang.plus(Math.toRadians(180));
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        taskRunner = new TaskRunner();
        drive = new MecanumDrive(hardwareMap, new Pose2d(-1.5 * TILE + 2.5, -2.5 * TILE - 5, Math.toRadians(0)));
        inp = new InputManager();
        inp.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> robot.getState().INTAKING && robot.extendo.hasSample(),
                () -> robot.extendo.setIntake(Extendo.INTAKE_OFF)
        ));
        Pose2d basket = new Pose2d(-2.37 * TILE, -2.37 * TILE, Math.toRadians(45));
        Pose2d first = new Pose2d(-2.5 * TILE + 2.5, -1 * TILE - 30, Math.toRadians(75));
        Pose2d firstPushed = forward(first, 5);
        Pose2d second = new Pose2d(-2.5 * TILE + 0.5, -1 * TILE - 31.7, Math.toRadians(90));
        Pose2d secondPushed = forward(second, 5);
        Pose2d third = new Pose2d(-2.5 * TILE + 6.5, -2.5 * TILE + 8, Math.toRadians(119.7));
        Pose2d thirdPushed = forward(third, 5);
        VelConstraint pushConstraint = new TranslationalVelConstraint(5);
        VelConstraint extendoConstraint = new AngularVelConstraint(Math.PI * 0.2);
        Task waitRobotTask = new Task().update(() -> robot.isTransitionDone());
        taskRunner.sendTask(robot.init());
        taskRunner.sendTask(actionToTask(drive.actionBuilder(drive.pose)
                .afterDisp(0, taskToAction(robot.transitionTask(Robot.Input.RIGHT_BUMPER)))
                .strafeToLinearHeading(basket.position, basket.heading)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))

                .afterDisp(0, taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .with(robot.extendo.setWrist(Extendo.WRIST_RAISED))
                        .with(robot.extendo.extend())))
                .strafeToSplineHeading(first.position, first.heading, extendoConstraint)
                .stopAndAdd(taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))))
                .splineTo(firstPushed.position, firstPushed.heading, pushConstraint)
                .afterDisp(0, taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))
                .strafeToSplineHeading(basket.position, basket.heading)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))

                .afterDisp(0, taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .with(robot.extendo.setWrist(Extendo.WRIST_RAISED))
                        .with(robot.extendo.extend())))
                .strafeToSplineHeading(second.position, second.heading, extendoConstraint)
                .stopAndAdd(taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))))
                .splineTo(secondPushed.position, secondPushed.heading, pushConstraint)
                .afterDisp(0, taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))
                .strafeToSplineHeading(basket.position, basket.heading)
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))

                .afterDisp(0, taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .with(robot.extendo.setWrist(Extendo.WRIST_RAISED))
                        .with(robot.extendo.extend())))
                .strafeToSplineHeading(third.position, third.heading, extendoConstraint)
                .stopAndAdd(taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))))
                .splineTo(thirdPushed.position, thirdPushed.heading, pushConstraint)
                .afterDisp(0, taskToAction(waitRobotTask
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))
                        .andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))
                .setReversed(true).splineTo(basket.position, flip(basket.heading))
                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.RIGHT_BUMPER))))

                .stopAndAdd(taskToAction(waitRobotTask.andThen(robot.transitionTask(Robot.Input.LEFT_BUMPER))))
                .build()
        ).resources(robot));
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
