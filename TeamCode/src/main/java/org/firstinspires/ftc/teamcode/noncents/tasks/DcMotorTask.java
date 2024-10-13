package org.firstinspires.ftc.teamcode.noncents.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class DcMotorTask implements Task {
    DcMotor motor;
    double target;

    public DcMotorTask(DcMotor motor, double target) {
        this.motor = motor;
        this.target = target;
    }

    @Override
    public boolean update() {
        return false;
    }

    @Override
    public Optional<Task> cancel() {
        return Optional.empty();
    }

    @Override
    public Set<String> getResources() {
        return new HashSet<>();
    }
}
