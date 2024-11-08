package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class Lift {
    public static final long WRIST_DELAY = 500;
    public static final double WRIST_TRANSFERRING = .965;
    public static final double WRIST_UP = 0.56;
    public static final double WRIST_DUMP = 0.1;

    public static final long CLAW_DELAY = 300;
    public static final double CLAW_CLOSED = 0.73;
    public static final double CLAW_OPENED = 0.92;

    public static final int MOTOR_MIN = 0;
    public static final int MOTOR_MAX = 3100;

    private final ServoWrapper liftWrist;
    private final ServoWrapper liftClaw;
    private final DcMotorEx liftMotor;

    boolean lock = true;

    public Lift(ServoWrapper liftWrist, ServoWrapper liftClaw, DcMotorEx liftMotor) {
        this.liftWrist = liftWrist;
        this.liftClaw = liftClaw;
        this.liftMotor = liftMotor;
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(MOTOR_MIN);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
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
            setMotorTargetPosition(MOTOR_MIN);
            lock = true;
        }).andThen(new Task().update(() -> !liftMotor.isBusy()));
    }

    public Task unlock() {
        return new Task().oneshot(() -> lock = false);
    }

    public void setMotorTargetPosition(int pos) {
        if (!lock) {
            int correctedPos = Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, pos));
            // less power on retract
            if (correctedPos == MOTOR_MIN) {
                liftMotor.setPower(.001);
            } else {
                liftMotor.setPower(1);
            }
            liftMotor.setTargetPosition(correctedPos);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private boolean isHolding;

    public void setMotorPower(double power) {
        if (power < 0) {
            power *= .1;
        }
        if (!lock) {
            boolean activelyMoving = Math.abs(power) > .01;
            boolean overshoot = activelyMoving && (getMotorCurrentPosition() > MOTOR_MAX && power > 0
                    || getMotorCurrentPosition() < MOTOR_MIN && power < 0);
            boolean needsToHold = !activelyMoving || overshoot;
            if (isHolding) {
                if (!needsToHold) {
                    isHolding = false;
                    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftMotor.setPower(power);
                }
            } else if (needsToHold) {
                isHolding = true;
                setMotorTargetPosition(Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, getMotorCurrentPosition())));
            } else {
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(power);
            }
        }
    }

    public int getMotorCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }
}
