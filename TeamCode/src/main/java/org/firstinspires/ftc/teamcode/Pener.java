package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TrajectoryTaskBuilder.actionToTask;
import static org.firstinspires.ftc.teamcode.TrajectoryTaskBuilder.taskToAction;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.noncents.input.InputManager;
import org.firstinspires.ftc.teamcode.noncents.input.Trigger;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@Autonomous
public class Pener extends OpMode {
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
        Pose2d start = new Pose2d(0, 0, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, start);
        inp = new InputManager();
        inp.addTrigger(new Trigger(
                Trigger.TriggerType.BEGIN,
                () -> robot.getState().INTAKING && robot.extendo.hasSample(),
                () -> robot.extendo.setIntake(Extendo.INTAKE_OFF)
        ));
        Pose2d basket = new Pose2d(-2.37 * TILE, -2.37 * TILE, Math.toRadians(45));
        Pose2d first = new Pose2d(-2 * TILE - 0.5, -2 * TILE - 9.7, Math.toRadians(90));
        Pose2d firstPushed = forward(first, 8);
        Pose2d second = new Pose2d(-2.5 * TILE + 2, -2 * TILE - 9.7, Math.toRadians(90));
        Pose2d secondPushed = forward(second, 8);
        Pose2d third = new Pose2d(-2.5 * TILE + 0.5, -2 * TILE - 7.2, Math.toRadians(95));
        Pose2d thirdPushed = forward(third, 6);
        Task waitRobotTask = new Task().update(() -> robot.isTransitionDone());
        Action waitRobot = taskToAction(waitRobotTask);
        taskRunner.sendTask(robot.init());
        taskRunner.sendTask(actionToTask(drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(0, 10), Math.toRadians(0))
                .splineTo(new Vector2d(10, 10), Math.toRadians(0))
                .build()
        ).resources(robot));
    }

    boolean first = true;
    boolean stopped = false;

    @Override
    public void loop() {
        telemetry.addData("state", robot.getState());
        telemetry.addData("lift", robot.lift.liftSetpoint);

        taskRunner.update();
        // TODO?
        // drive.updatePoseEstimate();
        robot.update();
    }
}
