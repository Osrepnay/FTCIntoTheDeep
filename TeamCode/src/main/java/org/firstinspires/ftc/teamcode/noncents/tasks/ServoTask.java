package org.firstinspires.ftc.teamcode.noncents.tasks;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class ServoTask implements Task {
    private Servo servo;
    private final double target;
    private final long delay;

    private final long startTime;
    private boolean firstUpdate = true;
    private Set<String> resources;

    public ServoTask(Servo servo, double target, long delay, String... resources) {
        this.servo = servo;
        this.target = target;
        this.delay = delay;
        this.startTime = System.currentTimeMillis();
        this.resources = new HashSet<>(Arrays.asList(resources));
    }

    @Override
    public boolean update() {
        if (firstUpdate) {
            firstUpdate = false;
            servo.setPosition(target);
        }
        return System.currentTimeMillis() - startTime >= delay;
    }

    @Override
    public Set<String> getResources() {
        return Collections.unmodifiableSet(resources);
    }
}
