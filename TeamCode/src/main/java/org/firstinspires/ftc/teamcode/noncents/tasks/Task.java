package org.firstinspires.ftc.teamcode.noncents.tasks;

import org.firstinspires.ftc.teamcode.noncents.Util;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Task {
    // TODO booleansupplier is not the best interface for this...
    public final BooleanSupplier update;
    public final boolean cancellable;
    public final Set<Object> resources;

    public Task() {
        this(() -> true, false, Collections.emptySet());
    }

    public Task(BooleanSupplier update, boolean cancellable, Set<Object> resources) {
        this.update = update;
        this.cancellable = cancellable;
        this.resources = resources;
    }

    public Task fromSupplier(Supplier<Task> taskSupplier) {
        Task[] task = new Task[1];
        return new Task().oneshot(() -> task[0] = taskSupplier.get())
                .andThen(new Task().update(task[0].update));
    }

    public Task update(BooleanSupplier update) {
        return new Task(update, this.cancellable, this.resources);
    }

    public Task oneshot(Runnable update) {
        return new Task(() -> {
            update.run();
            return true;
        }, this.cancellable, this.resources);
    }

    public Task continuous(Runnable update) {
        return new Task(() -> {
            update.run();
            return false;
        }, this.cancellable, this.resources);
    }

    public Task cancellable(boolean cancellable) {
        return new Task(this.update, cancellable, this.resources);
    }

    public Task resources(Set<Object> resources) {
        return new Task(this.update, this.cancellable, resources);
    }

    public Task resources(Object... resources) {
        return new Task(this.update, this.cancellable, Util.setOf(resources));
    }

    public Task withPrecondition(BooleanSupplier precond) {
        boolean[] condResult = {false};
        return new Task()
                .oneshot(() -> condResult[0] = precond.getAsBoolean())
                .andThen(new Task().update(() -> !condResult[0] || update.getAsBoolean()));
    }

    public Task andThen(Task next) {
        Set<Object> newResources = new HashSet<>(resources);
        newResources.addAll(next.resources);
        boolean[] thisIsDone = {false};
        return new Task(
                () -> {
                    if (thisIsDone[0]) {
                        return next.update.getAsBoolean();
                    } else {
                        if (this.update.getAsBoolean()) {
                            thisIsDone[0] = true;
                            return next.update.getAsBoolean();
                        }
                        return false;
                    }
                },
                this.cancellable && next.cancellable,
                Collections.unmodifiableSet(newResources)
        );
    }

    public Task with(Task with) {
        Set<Object> newResources = new HashSet<>(resources);
        newResources.addAll(with.resources);
        boolean[] doneness = {false, false};
        return new Task(
                () -> {
                    if (!doneness[0] && this.update.getAsBoolean()) {
                        doneness[0] = true;
                    }
                    if (!doneness[1] && with.update.getAsBoolean()) {
                        doneness[1] = true;
                    }
                    return doneness[0] && doneness[1];
                },
                this.cancellable && with.cancellable,
                Collections.unmodifiableSet(newResources)
        );
    }

    // TODO this is bad, we can't have resource copying or whatnot with this
    public static Task defer(Supplier<Task> supplier) {
        Task[] tasks = new Task[1];
        return new Task()
                .oneshot(() -> tasks[0] = supplier.get())
                .andThen(new Task().update(() -> tasks[0].update.getAsBoolean()));
    }

    public static Task delay(long ms) {
        long[] start = {-1};
        return new Task()
                .update(() -> {
                    if (start[0] == -1) {
                        start[0] = System.currentTimeMillis();
                    }
                    return System.currentTimeMillis() - start[0] >= ms;
                });
    }
}
