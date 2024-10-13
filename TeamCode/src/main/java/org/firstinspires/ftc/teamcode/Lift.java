package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public enum WristPosition {
        TRANSFER(0),
        SPECIMEN(0),
        SAMPLE(0);

        public final double servoPos;

        WristPosition(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    public enum ClawPosition {
        CLOSED(0),
        OPEN(0);

        public final double servoPos;

        ClawPosition(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    private final DcMotor lift;
    private final Servo liftWrist;
    private final Servo liftClaw;

    private int liftTargetPosition = 0;

    public Lift(DcMotor lift, Servo liftWrist, Servo liftClaw) {
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift = lift;
        this.liftWrist = liftWrist;
        this.liftClaw = liftClaw;
    }

    public void setWrist(WristPosition pos) {
        liftWrist.setPosition(pos.servoPos);
    }

    public void setClaw(ClawPosition pos) {
        liftClaw.setPosition(pos.servoPos);
    }

    public void setLiftTargetPosition(int pos) {
        liftTargetPosition = Math.max(0, Math.min(100, pos));
        lift.setTargetPosition(liftTargetPosition);
        if (Math.abs(lift.getPower()) < 0.002) {
            lift.setPower(0.1);
        }
    }

    public int getLiftTargetPosition() {
        return liftTargetPosition;
    }
}
