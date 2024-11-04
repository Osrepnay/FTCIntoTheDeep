package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

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

    private final ServoWrapper liftWrist;
    private final ServoWrapper liftClaw;
    private final DcMotorEx liftMotor;

    private boolean lock = true;

    public Lift(ServoWrapper liftWrist, ServoWrapper liftClaw, DcMotorEx liftMotor) {
        this.liftWrist = liftWrist;
        this.liftClaw = liftClaw;
        this.liftMotor = liftMotor;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Lift(HardwareMap hardwareMap) {
        this(
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "liftWrist"), WRIST_DELAY),
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "liftClaw"), CLAW_DELAY),
                hardwareMap.get(DcMotorEx.class, "liftMotor")
        );
    }

    public Task setWrist(double pos) {
        return liftWrist.setPosition(pos);
    }

    public Task setClaw(double pos) {
        return liftClaw.setPosition(pos);
    }

    public Task retract() {
        return new Task().oneshot(() -> {
            setMotorPos(0);
            lock = true;
        }).andThen(new Task().update(() -> !liftMotor.isBusy()));
    }

    public Task unlock() {
        return new Task().oneshot(() -> lock = false);
    }

    public void setMotorPos(int pos) {
        if (!lock) {
            int correctedPos = Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, pos));
            liftMotor.setTargetPosition(correctedPos);
        }
    }

    public int getMotorPos() {
        return liftMotor.getTargetPosition();
    }
}
