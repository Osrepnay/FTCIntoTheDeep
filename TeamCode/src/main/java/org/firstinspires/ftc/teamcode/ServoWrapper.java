package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Optional;

public class ServoWrapper {
    private long lastChange = -1;
    private double lastChangeTo;
    public final ServoImplEx servo;
    public final long moveDelay;

    public ServoWrapper(ServoImplEx servo, long moveDelay) {
        this.servo = servo;
        this.moveDelay = moveDelay;
    }

    public Task setPosition(double pos) {
        Task delayTask = Task.delay(moveDelay);
        boolean[] first = {true};
        return new Task()
                .update(() -> {
                    if (getPosition().map(p -> p == pos).orElse(false)) {
                        return true;
                    } else {
                        if (first[0]) {
                            lastChange = System.currentTimeMillis();
                            lastChangeTo = pos;
                            servo.setPosition(pos);
                            first[0] = true;
                        }
                        return delayTask.update.getAsBoolean();
                    }
                });
    }

    // returns none if current position is unknown
    public Optional<Double> getPosition() {
        // no position has been set, skip
        if (lastChange == -1) {
            return Optional.empty();
        }

        if (System.currentTimeMillis() - lastChange >= moveDelay) {
            return Optional.of(lastChangeTo);
        } else {
            return Optional.empty();
        }
    }
}
