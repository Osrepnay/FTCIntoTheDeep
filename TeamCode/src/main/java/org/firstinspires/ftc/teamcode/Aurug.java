package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;
import org.firstinspires.ftc.teamcode.noncents.tasks.TaskRunner;

@Autonomous
public class Aurug extends OpMode {
    Robot robot;
    TaskRunner taskRunner;
    MecanumDrive drive;

    private final double TILE = 29;

    private Action taskToAction(Task task) {
        return _telemetryPacket -> !task.update.getAsBoolean();
    }

    private Task actionToTask(Action action) {
        TelemetryPacket trash = new TelemetryPacket();
        return new Task().update(() -> !action.run(trash));
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        taskRunner = new TaskRunner();
        drive = new MecanumDrive(hardwareMap, new Pose2d(-1.5 * TILE, -2.5 * TILE - 3, Math.toRadians(90)));
        TrajectoryActionBuilder toFirst = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(-2 * TILE - 1, -1.5 * TILE), Math.toRadians(90))
                .splineTo(new Vector2d(-2 * TILE - 1, -1 * TILE - 7), Math.toRadians(90));
        TrajectoryActionBuilder toBasketFirst = toFirst.fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-2.5 * TILE, -2.2 * TILE), Math.toRadians(225));
        TrajectoryActionBuilder firstBasketRetreat = toBasketFirst.fresh()
                .setReversed(false)
                .splineTo(new Vector2d(-2.5 * TILE + 3, -2.2 * TILE + 3), Math.toRadians(45));
        TrajectoryActionBuilder toSecond = toBasketFirst.fresh()
                .setReversed(false)
                .splineTo(new Vector2d(-2.5 * TILE, -1 * TILE - 7), Math.toRadians(90));
        TrajectoryActionBuilder toBasketSecond = toSecond.fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-2.5 * TILE, -2.2 * TILE), Math.toRadians(225));
        TrajectoryActionBuilder secondBasketRetreat = toBasketSecond.fresh()
                .setReversed(false)
                .splineTo(new Vector2d(-2.5 * TILE + 3, -2.2 * TILE + 3), Math.toRadians(45));
        taskRunner.sendTask(actionToTask(new SequentialAction(
                taskToAction(
                        robot.transition(Robot.State.TRANSFER, Robot.State.EXTENDING).get()
                                .andThen(robot.transition(Robot.State.EXTENDING, Robot.State.RUMMAGE).get())
                ),
                toFirst.build(),
                taskToAction(
                        robot.transition(Robot.State.RUMMAGE, Robot.State.EXTENDING).get()
                                .andThen(robot.transition(Robot.State.EXTENDING, Robot.State.TRANSFER).get())
                                .andThen(robot.transition(Robot.State.TRANSFER, Robot.State.LIFTING).get())
                                .andThen(new Task().oneshot(() -> robot.lift.setMotorTargetPosition(Lift.MOTOR_MAX)))
                ),
                toBasketFirst.build(),
                taskToAction(robot.lift.motorWait().andThen(robot.dump())),
                firstBasketRetreat.build(),
                taskToAction(
                        robot.transition(Robot.State.LIFTING, Robot.State.TRANSFER).get()
                            .andThen(robot.transition(Robot.State.TRANSFER, Robot.State.EXTENDING).get())
                            .andThen(robot.transition(Robot.State.EXTENDING, Robot.State.RUMMAGE).get())
                ),
                toSecond.build(),
                taskToAction(
                        robot.transition(Robot.State.RUMMAGE, Robot.State.EXTENDING).get()
                                .andThen(robot.transition(Robot.State.EXTENDING, Robot.State.TRANSFER).get())
                                .andThen(robot.transition(Robot.State.TRANSFER, Robot.State.LIFTING).get())
                                .andThen(new Task().oneshot(() -> robot.lift.setMotorTargetPosition(Lift.MOTOR_MAX)))
                ),
                toBasketSecond.build(),
                taskToAction(robot.lift.motorWait().andThen(robot.dump())),
                secondBasketRetreat.build(),
                taskToAction(
                        robot.transition(Robot.State.LIFTING, Robot.State.TRANSFER).get()
                                .andThen(robot.transition(Robot.State.TRANSFER, Robot.State.EXTENDING).get())
                                .andThen(robot.transition(Robot.State.EXTENDING, Robot.State.RUMMAGE).get())
                )
        )));
    }

    boolean first = true;
    boolean stopped = false;

    @Override
    public void loop() {
        if (first) {
            first = false;
            taskRunner.sendTask(robot.init());
        }

        taskRunner.update();
        // TODO?
        drive.updatePoseEstimate();
    }
}
