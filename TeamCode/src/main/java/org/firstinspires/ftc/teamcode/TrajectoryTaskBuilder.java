package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.ArrayList;
import java.util.List;
import java.util.function.UnaryOperator;

public class TrajectoryTaskBuilder {
    private TrajectoryActionBuilder lastBuilder;
    private List<Task> tasks = new ArrayList<>();

    public static Action taskToAction(Task task) {
        return _telemetryPacket -> !task.update.getAsBoolean();
    }

    public static Task actionToTask(Action action) {
        TelemetryPacket trash = new TelemetryPacket();
        return new Task().update(() -> !action.run(trash));
    }

    public TrajectoryTaskBuilder(TrajectoryActionBuilder tab) {
        lastBuilder = tab;
    }

    public TrajectoryTaskBuilder andThen(Task t) {
        tasks.add(t);
        return this;
    }

    public TrajectoryTaskBuilder andThen(UnaryOperator<TrajectoryActionBuilder> tab) {
        lastBuilder = tab.apply(lastBuilder);
        tasks.add(actionToTask(lastBuilder.build()));
        return this;
    }

    public TrajectoryTaskBuilder withLast(Task t) {
        tasks.set(tasks.size() - 1, tasks.get(tasks.size() - 1).with(t));
        return this;
    }

    public Task build() {
        return tasks.stream().reduce(new Task(), Task::andThen);
    }

}
