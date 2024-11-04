package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

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

    private final ServoWrapper extendoWrist;
    private final ServoWrapper extendoClaw;
    private final DcMotorEx extendoMotor;

    private boolean lock = false;

    public Extendo(ServoWrapper extendoWrist, ServoWrapper extendoClaw, DcMotorEx extendoMotor) {
        this.extendoWrist = extendoWrist;
        this.extendoClaw = extendoClaw;
        this.extendoMotor = extendoMotor;
        extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            extendoMotor.setPower(0.5);
            setMotorPos(0);
            lock = true;
        }).andThen(new Task().update(() -> !extendoMotor.isBusy()));
    }

    public Task unlock() {
        return new Task().oneshot(() -> lock = false);
    }

    public void setMotorPos(int pos) {
        if (!lock) {
            int correctedPos = Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, pos));
            extendoMotor.setTargetPosition(correctedPos);
        }
    }

    public int getMotorPos() {
        return extendoMotor.getTargetPosition();
    }

}