package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
        drive = new MecanumDrive(hardwareMap, new Pose2d(-2.5 * TILE, 1.5 * TILE - 2, 0));
        Action trajectory = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(-1.5 * TILE, 2 * TILE), 0)
                .splineTo(new Vector2d(-1.5 * TILE + 3, 2 * TILE), 0)
                /* .stopAndAdd(taskToAction(
                        robot.toState(Robot.State.EXTENDING).get()
                                .andThen(robot.toState(Robot.State.RUMMAGE).get()))) */
                .endTrajectory()
                .splineTo(new Vector2d(-2.5 * TILE + 5, 2.5 * TILE - 5), Math.toRadians(-45))
                .endTrajectory()
                /* .stopAndAdd(taskToAction(
                        robot.toState(Robot.State.EXTENDING).get()
                                .andThen(robot.toState(Robot.State.LIFTING).get())
                                .andThen(new Task().oneshot(() -> robot.lift.setMotorTargetPosition(Lift.MOTOR_MAX)))
                                .andThen(robot.lift.motorWait())
                                .andThen(robot.dump()))) */
                .splineTo(new Vector2d(-2 * TILE, -2 * TILE), Math.toRadians(-45))
                .build();
        taskRunner.sendTask(actionToTask(trajectory));
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
