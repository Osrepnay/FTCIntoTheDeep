package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class Extendo {
    public static final long WRIST_DELAY = 1400;
    public static final double WRIST_TRANSFERRING = 0.735;
    public static final double WRIST_RAISED = 0.18;
    public static final double WRIST_SPEC = 0.105;
    public static final double WRIST_SAMPLE = 0.056;

    // public static final long INTAKE_DELAY = 300;
    public static final double INTAKE_OFF = 0;
    public static final double INTAKE_ON = 1;
    public static final double INTAKE_VOMIT = -1;

    public static final long EXTENDO_DELAY = 1200;

    public static final double LEFT_RETRACTED = 0.492;
    public static final double LEFT_EXTENDED = 0.73;

    public static final double RIGHT_RETRACTED = 0.48;
    public static final double RIGHT_EXTENDED = 0.235;

    private final ServoWrapper extendoWrist;
    private final CRServo extendoIntake;
    private final ServoWrapper extendoLeft;
    private final ServoWrapper extendoRight;
    private final DistanceSensor extendoDist;

    public Extendo(ServoWrapper extendoWrist, CRServo extendoIntake, ServoWrapper extendoLeft,
                   ServoWrapper extendoRight, DistanceSensor extendoDist) {
        this.extendoWrist = extendoWrist;
        this.extendoIntake = extendoIntake;
        this.extendoLeft = extendoLeft;
        this.extendoRight = extendoRight;
        this.extendoDist = extendoDist;
    }

    public Extendo(HardwareMap hardwareMap) {
        this(
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "extendoPivot"), WRIST_DELAY),
                hardwareMap.get(CRServo.class, "intake"),
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "extendoLeft"), EXTENDO_DELAY),
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "extendoRight"), EXTENDO_DELAY),
                hardwareMap.get(DistanceSensor.class, "extendoDist")
        );
    }

    public Task setWrist(double pos) {
        return extendoWrist.setPosition(pos).resources(this);
    }

    public Task setIntake(double power) {
        return new Task().oneshot(() -> extendoIntake.setPower(power)).resources(this);
    }

    public Task retract() {
        return extendoLeft.setPosition(LEFT_RETRACTED).with(extendoRight.setPosition(RIGHT_RETRACTED)).resources(this);
    }

    public Task extendHalf() {
        return extendoLeft.setPosition((LEFT_EXTENDED + LEFT_RETRACTED) / 2).with(extendoRight.setPosition((RIGHT_EXTENDED + RIGHT_RETRACTED) / 2)).resources(this);
    }

    public Task extend() {
        return extendoLeft.setPosition(LEFT_EXTENDED).with(extendoRight.setPosition(RIGHT_EXTENDED)).resources(this);
    }

    public Task spew() {
        double[] oldIntake = {0};
        return new Task().oneshot(() -> oldIntake[0] = extendoIntake.getPower())
                .andThen(setIntake(INTAKE_VOMIT))
                .andThen(Task.delay(500))
                .andThen(Task.defer(() -> setIntake(oldIntake[0])))
                .resources(this);
    }

    public boolean hasSample() {
        return extendoDist.getDistance(DistanceUnit.CM) < 5;
    }

    public boolean hasCorrectSample() {
        /*
        int[] colors = {extendoColor.red(), extendoColor.green(), extendoColor.blue()};
        int maxIdx = 0;
        for (int i = 1; i < colors.length; i++) {
            if (colors[i] > colors[maxIdx]) {
                maxIdx = i;
            }
        }
        boolean colorValid = maxIdx == 1
                || (Color.currentColor == Color.RED
                ? maxIdx == 0
                : maxIdx == 2);
         */
        return hasSample();
    }

}