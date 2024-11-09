package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class Extendo {
    public static final long WRIST_DELAY = 500;
    public static final double WRIST_TRANSFERRING = 0.7;
    public static final double WRIST_RAISED = 0.35;
    public static final double WRIST_LOWERED = 0.13;

    public static final long CLAW_DELAY = 300;
    public static final double CLAW_CLOSED = 0.56;
    public static final double CLAW_OPENED = 0.45;

    public static final int MOTOR_MIN = 0;
    public static final int MOTOR_MAX = 3000;

    private final ServoWrapper extendoWrist;
    private final ServoWrapper extendoClaw;
    private final DcMotorEx extendoMotor;

    private boolean lock = false;

    public Extendo(ServoWrapper extendoWrist, ServoWrapper extendoClaw, DcMotorEx extendoMotor) {
        this.extendoWrist = extendoWrist;
        this.extendoClaw = extendoClaw;
        this.extendoMotor = extendoMotor;
        extendoMotor.setDirection(DcMotor.Direction.REVERSE);
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setTargetPosition(MOTOR_MIN);
        extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendoMotor.setPower(1);
    }

    public Extendo(HardwareMap hardwareMap) {
        this(
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "extendoWrist"), WRIST_DELAY),
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "extendoClaw"), CLAW_DELAY),
                hardwareMap.get(DcMotorEx.class, "extendoMotor")
        );
    }

    public Task setWrist(double pos) {
        return extendoWrist.setPosition(pos);
    }

    public Task setClaw(double pos) {
        return extendoClaw.setPosition(pos);
    }

    public Task retract() {
        return new Task().oneshot(() -> {
            setMotorTargetPosition(MOTOR_MIN);
            lock = true;
        }).andThen(new Task().update(() -> !extendoMotor.isBusy()));
    }

    public Task unlock() {
        return new Task().oneshot(() -> lock = false);
    }

    public void setMotorTargetPosition(int pos) {
        if (!lock) {
            int correctedPos = Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, pos));
            extendoMotor.setTargetPosition(correctedPos);
            extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendoMotor.setPower(1);
        }
    }

    public void setMotorInput(double input) {
        if (lock) {
            return;
        }

        int motorPos = getMotorCurrentPosition();
        int slowdownRange = 300;
        double stallPower = 0.1;
        int maxDist = MOTOR_MAX - motorPos;
        if (maxDist < slowdownRange && input > 0) {
            input *= Math.max(0, (double) maxDist * (1 - stallPower) / slowdownRange + stallPower);
        }
        int minDist = motorPos - MOTOR_MIN;
        if (minDist < slowdownRange && input < 0) {
            input *= Math.max(0, (double) minDist * (1 - stallPower) / slowdownRange + stallPower);
        }
        setMotorPower(input);
    }

    public void setMotorPower(double power) {
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoMotor.setPower(power);
    }

    public int getMotorCurrentPosition() {
        return extendoMotor.getCurrentPosition();
    }

}