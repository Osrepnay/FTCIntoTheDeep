package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Collections;

public class Extendo {
    public static final long WRIST_DELAY = 300;
    public static final double WRIST_TRANSFERRING = 0;
    public static final double WRIST_RAISED = 0;
    public static final double WRIST_LOWERED = 0;

    public static final long CLAW_DELAY = 100;
    public static final double CLAW_CLOSED = 0;
    public static final double CLAW_OPENED = 0;

    public static final int MOTOR_MIN = 0;
    public static final int MOTOR_MAX = 100;

    private int motorPos = 0;

    private final ServoImplEx extendoWrist;
    private final ServoImplEx extendoClaw;
    private final DcMotorEx extendoMotor;

    public Extendo(ServoImplEx extendoWrist, ServoImplEx extendoClaw, DcMotorEx extendoMotor) {
        this.extendoWrist = extendoWrist;
        this.extendoClaw = extendoClaw;
        this.extendoMotor = extendoMotor;
        extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Extendo(HardwareMap hardwareMap) {
        this(
                hardwareMap.get(ServoImplEx.class, "extendoWrist"),
                hardwareMap.get(ServoImplEx.class, "extendoClaw"),
                hardwareMap.get(DcMotorEx.class, "extendoMotor")
        );
    }

    public void setWrist(double pos) {
        extendoWrist.setPosition(pos);
    }

    public void setClaw(double pos) {
        extendoClaw.setPosition(pos);
    }

    public void setMotorPos(int pos) {
        int correctedPos = Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, pos));
        extendoMotor.setTargetPosition(correctedPos);
    }

    public int getMotorPos() {
        return extendoMotor.getTargetPosition();
    }

    public Task toTransfer() {
        return new Task(() -> setClaw(CLAW_CLOSED), Collections.emptySet())
                .with(new DelayTask(CLAW_DELAY))
                .andThen(new Task(() -> setWrist(WRIST_TRANSFERRING), Collections.emptySet())
                        .with(new DelayTask(WRIST_DELAY)))
                .andThen(new Task(() -> {
                    setMotorPos(0);
                    return !extendoMotor.isBusy();
                }, Collections.emptySet()))
                .with(new Task(() -> {}, Collections.singleton(this)));
    }

    public Task toExtend() {
        return new Task(() -> setClaw(CLAW_CLOSED), Collections.emptySet())
                .with(new DelayTask(CLAW_DELAY))
                .andThen(new Task(() -> setWrist(WRIST_RAISED), Collections.emptySet())
                        .with(new DelayTask(WRIST_DELAY)));
    }

    public Task toRummage() {
        return new Task(() -> setClaw(CLAW_OPENED), Collections.emptySet())
                .with(new DelayTask(CLAW_DELAY))
                .andThen(new Task(() -> setWrist(WRIST_LOWERED), Collections.emptySet())
                .with(new DelayTask(WRIST_DELAY)));
    }
}