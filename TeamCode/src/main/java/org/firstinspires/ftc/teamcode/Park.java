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
public class Park extends OpMode {
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
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        TrajectoryActionBuilder toFirst = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(TILE * 1.5, 0), Math.toRadians(0));
        taskRunner.sendTask(actionToTask(new SequentialAction(
                toFirst.build()
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
