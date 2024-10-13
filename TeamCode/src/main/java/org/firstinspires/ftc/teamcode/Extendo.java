package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo {
    public enum WristPosition {
        DOWN(0),
        CLEARANCE(0),
        TRANSFER(0);

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

    private int extendoTargetPosition = 0;

    private final DcMotor extendo;
    private final Servo extendoWrist;
    private final Servo extendoClaw;

    public Extendo(DcMotor extendo, Servo extendoWrist, Servo extendoClaw) {
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.extendo = extendo;
        this.extendoWrist = extendoWrist;
        this.extendoClaw = extendoClaw;
    }

    public void setWrist(WristPosition pos) {
        extendoWrist.setPosition(pos.servoPos);
    }

    public void setClaw(ClawPosition pos) {
        extendoClaw.setPosition(pos.servoPos);
    }

    public void setExtendoTargetPosition(int pos) {
        extendoTargetPosition = Math.max(0, Math.min(100, pos));
        extendo.setTargetPosition(extendoTargetPosition);
        if (Math.abs(extendo.getPower()) < 0.002) {
            extendo.setPower(0.1);
        }
    }

    public int getExtendoTargetPosition() {
        return extendoTargetPosition;
    }
}
