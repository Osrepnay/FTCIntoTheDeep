package org.firstinspires.ftc.teamcode.noncents.tasks;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class Task {
    // TODO booleansupplier is not the best interface for this...
    public final BooleanSupplier update;
    public final Set<String> resources;

    private Task(BooleanSupplier update, Set<String> resources) {
        this.update = update;
        this.resources = resources;
    }

    public Task(BooleanSupplier update, String... resources) {
        this.update = update;
        this.resources = Collections.unmodifiableSet(new HashSet<>(Arrays.asList(resources)));
    }

    public Task(Runnable update, String... resources) {
        this.update = () -> {
            update.run();
            return true;
        };
        this.resources = Collections.unmodifiableSet(new HashSet<>(Arrays.asList(resources)));
    }

    public Task andThen(Task next) {
        Set<String> newResources = new HashSet<>(resources);
        newResources.addAll(next.resources);
        boolean[] thisIsDone = {false};
        return new Task(
                () -> {
                    if (thisIsDone[0]) {
                        return next.update.getAsBoolean();
                    } else {
                        if (this.update.getAsBoolean()) {
                            thisIsDone[0] = true;
                        }
                        return false;
                    }
                },
                Collections.unmodifiableSet(newResources)
        );
    }

    public Task with(Task with) {
        Set<String> newResources = new HashSet<>(resources);
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
                Collections.unmodifiableSet(newResources)
        );
    }
}
