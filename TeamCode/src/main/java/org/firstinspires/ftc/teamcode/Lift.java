package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.noncents.PIDController;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Lift {
    public static final long WRIST_DELAY = 1100;
    public static final double WRIST_TRANSFERRING = 0.99;
    public static final double WRIST_LIFT = 0.50;
    public static final double WRIST_DUMP = 0.36;
    public static final double WRIST_SPEC_SCORE = 0.25;
    public static final double WRIST_SPEC_PICKUP = 0.15;

    public static final long CLAW_DELAY = 1500;
    public static final double CLAW_CLOSED = 0.447;
    public static final double CLAW_OPENED = 0.56;

    public static final int MOTOR_MIN = 0;
    public static final int MOTOR_SPECIMEN_PICKUP = 40;
    public static final int MOTOR_SPECIMEN_SCORE = 965;
    public static final int MOTOR_MAX = 2400;

    public final int LIFT_TOLERANCE_TICKS = 10;

    public static final double OFFSET_STEP = (2400.0 - 600) / (2500 - 500) * 300 / 360 / 24;

    private double offset = 0;
    private double lastSet = 0;

    private final ServoWrapper liftWrist;
    private final ServoWrapper liftClaw;
    private final DcMotorEx liftMotorLeft;
    private final DcMotorEx liftMotorRight;
    private final VoltageSensor voltageSensor;

    public static PIDController pid = new PIDController(0.006, 0.00002, 0.3, 20000, 0.8, 100);
    private final double HOLD_MAX = 0.07;

    private boolean pidControlled = true;
    public double liftSetpoint = MOTOR_MIN;
    private long last = System.currentTimeMillis();

    public Lift(ServoWrapper liftWrist, ServoWrapper liftClaw, DcMotorEx liftMotorLeft, DcMotorEx liftMotorRight,
                VoltageSensor voltageSensor) {
        this.liftWrist = liftWrist;
        this.liftClaw = liftClaw;
        this.liftMotorLeft = new CachingDcMotorEx(liftMotorLeft);
        this.liftMotorRight = new CachingDcMotorEx(liftMotorRight);
        this.voltageSensor = voltageSensor;
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Lift(HardwareMap hardwareMap) {
        this(
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "liftPivot"), WRIST_DELAY),
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "liftClaw"), CLAW_DELAY),
                hardwareMap.get(DcMotorEx.class, "liftLeft"),
                hardwareMap.get(DcMotorEx.class, "liftRight"),
                hardwareMap.get(VoltageSensor.class, "Control Hub")
        );
    }

    public void update() {
        if (pidControlled) {
            double power = pid.update(liftSetpoint, liftMotorLeft.getCurrentPosition());
            power += HOLD_MAX * liftSetpoint / 2400;
            double voltage = voltageSensor.getVoltage();
            power *= 14 / voltage;
            liftMotorLeft.setPower(power);
            liftMotorRight.setPower(power);
        }
    }

    public Task setWrist(double pos) {
        return liftWrist.setPosition(pos + offset).with(new Task().oneshot(() -> lastSet = pos)).resources(this);
    }

    public Task setClaw(double pos) {
        return liftClaw.setPosition(pos).resources(this);
    }

    public Task liftTo(int pos) {
        return new Task().oneshot(() -> {
            setMotorTargetPosition(pos);
        }).andThen(motorWait()).resources(this);
    }

    public Task retract() {
        return liftTo(MOTOR_MIN);
    }

    public void setMotorTargetPosition(int pos) {
        liftSetpoint = Math.min(MOTOR_MAX, Math.max(MOTOR_MIN, pos));
        pid.reset();
        pidControlled = true;
    }

    private boolean isHolding = true;

    public void setMotorInput(double input) {
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

        /*
        int slowdownRange = 150;
        double stallPower = 0.8;
        int maxDist = MOTOR_MAX - motorPos;
        if (maxDist < slowdownRange && input > 0) {
            input *= Math.max(0, (double) maxDist * (1 - stallPower) / slowdownRange + stallPower);
        }
         */
        /*
        int minDist = Math.abs(motorPos - MOTOR_MIN);
        if (minDist < slowdownRange && input < 0) {
            input *= (double) minDist * (1 - stallPower) / slowdownRange + stallPower;
        }
        */
        // gravity
        /*
        if (input < 0) {
            input *= .65;
        }
         */
        setMotorPower(input);
    }

    public void setMotorPower(double power) {
        pidControlled = false;
        liftMotorLeft.setPower(power);
        liftMotorRight.setPower(power);
    }

    public int getMotorCurrentPosition() {
        return liftMotorLeft.getCurrentPosition();
    }

    public void resetMotor() {
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Task motorWait() {
        return new Task().update(() -> !pidControlled
                        || Math.abs(getMotorCurrentPosition() - liftSetpoint) <= LIFT_TOLERANCE_TICKS)
                .resources(this);
    }

    public Task addOffset(double offsetChange) {
        offset += offsetChange;
        return setWrist(lastSet);
    }

    public double getOffset() {
        return offset;
    }

    public void resetOffset() {
        offset = 0;
    }
}
