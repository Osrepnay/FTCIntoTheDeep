package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.tasks.DelayTask;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import java.util.Collections;

public class Lift {
    public static final long WRIST_DELAY = 300;
    public static final double WRIST_TRANSFERRING = 0;
    public static final double WRIST_UP = 0;
    public static final double WRIST_DUMP = 0;

    public static final long CLAW_DELAY = 100;
    public static final double CLAW_CLOSED = 0;
    public static final double CLAW_OPENED = 0;

    public static final int MOTOR_MIN = 0;
    public static final int MOTOR_MAX = 100;

    private int motorPos = 0;

    private final ServoImplEx liftWrist;
    private final ServoImplEx liftClaw;
    private final DcMotorEx liftMotor;

    public Lift(ServoImplEx liftWrist, ServoImplEx liftClaw, DcMotorEx liftMotor) {
        this.liftWrist = liftWrist;
        this.liftClaw = liftClaw;
        this.liftMotor = liftMotor;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Lift(HardwareMap hardwareMap) {
        this(
                hardwareMap.get(ServoImplEx.class, "liftWrist"),
                hardwareMap.get(ServoImplEx.class, "liftClaw"),
                hardwareMap.get(DcMotorEx.class, "liftMotor")
        );
    }

    public void setWrist(double pos) {
        liftWrist.setPosition(pos);
    }

    public void setClaw(double pos) {
        liftClaw.setPosition(pos);
    }

    public void setMotorPos(int pos) {
        int correctedPos = Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, pos));
        liftMotor.setTargetPosition(correctedPos);
    }

    public int getMotorPos() {
        return liftMotor.getTargetPosition();
    }

    public Task toTransfer() {
        return new Task(() -> setClaw(CLAW_CLOSED), Collections.emptySet())
                .with(new DelayTask(CLAW_DELAY))
                .andThen(new Task(() -> setWrist(WRIST_TRANSFERRING), Collections.emptySet())
                        .with(new DelayTask(WRIST_DELAY)))
                .andThen(new Task(() -> {
                    setMotorPos(0);
                    return !liftMotor.isBusy();
                }, Collections.emptySet()))
                .with(new Task(() -> {}, Collections.singleton(this)));
    }

    public Task toLift() {
        return new Task(() -> setClaw(CLAW_CLOSED), Collections.emptySet())
                .with(new DelayTask(CLAW_DELAY))
                .andThen(new Task(() -> setWrist(WRIST_UP), Collections.emptySet())
                        .with(new DelayTask(WRIST_DELAY)));
    }

    public Task dump() {
        return new Task(() -> setWrist(WRIST_DUMP), Collections.emptySet())
                .with(new DelayTask(WRIST_DELAY))
                .andThen(new Task(() -> setClaw(CLAW_OPENED), Collections.emptySet())
                        .with(new DelayTask(CLAW_DELAY)))
                .andThen(new Task(() -> setWrist(WRIST_UP), Collections.emptySet())
                    .with(new DelayTask(WRIST_DELAY)))
                .andThen(new Task(() -> setClaw(CLAW_CLOSED), Collections.emptySet())
                        .with(new DelayTask(CLAW_DELAY)));
    }
}
