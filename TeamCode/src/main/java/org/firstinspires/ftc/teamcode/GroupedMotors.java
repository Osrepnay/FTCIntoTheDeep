package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.noncents.PIDController;

public class GroupedMotors {
    private DcMotorEx[] motors;
    private DcMotorEx encoder;

    private PIDController pid;

    private double power = 0;

    public GroupedMotors(DcMotorEx[] motors, DcMotorEx encoder, PIDController pid) {
        this.motors = motors;
        this.encoder = encoder;
        this.pid = pid;
    }

}
