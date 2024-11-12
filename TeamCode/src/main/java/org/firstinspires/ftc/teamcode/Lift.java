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
    public static final int MOTOR_MAX = 3000;

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
        System.out.println("elsjliejfs");
        return liftWrist.setPosition(pos);
    }

    public Task setClaw(double pos) {
        return liftClaw.setPosition(pos);
    }

    public Task retract() {
        return new Task().oneshot(() -> {
            setMotorTargetPosition(MOTOR_MIN);
            lock = true;
        }).andThen(motorWait());
    }

    public Task unlock() {
        return new Task().oneshot(() -> lock = false);
    }

    public void setMotorTargetPosition(int pos) {
        if (!lock) {
            int correctedPos = Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, pos));
            liftMotor.setPower(1);
            liftMotor.setTargetPosition(correctedPos);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private boolean isHolding;

    public void setMotorInput(double input) {
        if (lock) {
            return;
        }

        int motorPos = getMotorCurrentPosition();
        if (Math.abs(input) < 0.01) {
            if (!isHolding) {
                isHolding = true;
                setMotorTargetPosition(motorPos);
            }
            return;
        } else {
            isHolding = false;
        }

        int slowdownRange = 150;
        double stallPower = 0.8;
        int maxDist = MOTOR_MAX - motorPos;
        if (maxDist < slowdownRange && input > 0) {
            input *= Math.max(0, (double) maxDist * (1 - stallPower) / slowdownRange + stallPower);
        }
        /*
        int minDist = Math.abs(motorPos - MOTOR_MIN);
        if (minDist < slowdownRange && input < 0) {
            input *= (double) minDist * (1 - stallPower) / slowdownRange + stallPower;
        }
        */
        // gravity
        if (input < 0) {
            input *= .65;
        }
        setMotorPower(input);
    }

    public void setMotorPower(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(power);
    }

    public int getMotorCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public Task motorWait() {
        return new Task().update(() -> !liftMotor.isBusy());
    }
}
