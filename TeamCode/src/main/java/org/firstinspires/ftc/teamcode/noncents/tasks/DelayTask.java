package org.firstinspires.ftc.teamcode.noncents.tasks;

import java.util.function.BooleanSupplier;

public class DelayTask extends Task {
    public final long delay;

    public DelayTask(long delay) {
        super(new BooleanSupplier() {
            private final long start = System.currentTimeMillis();

            @Override
            public boolean getAsBoolean() {
                return System.currentTimeMillis() - start >= delay;
            }
        });
        this.delay = delay;
    }
}