package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.noncents.tasks.Task;

public class Extendo {
    public static final long WRIST_DELAY = 1400;
    public static final double WRIST_TRANSFERRING = 0.85;
    public static final double WRIST_RAISED = 0.19;
    public static final double WRIST_STAGE = 0.35;
    public static final double WRIST_SAMPLE = 0.10;

    // public static final long INTAKE_DELAY = 300;
    public static final double INTAKE_OFF = 0;
    public static final double INTAKE_ON = 1;
    public static final double INTAKE_VOMIT = -1;

    public static final long EXTENDO_DELAY = 1200;

    public static final double LEFT_RETRACTED = 0.165;
    public static final double LEFT_EXTENDED = 0.585;

    public static final double RIGHT_RETRACTED = 0.79;
    public static final double RIGHT_EXTENDED = 0.38;

    public static final double OFFSET_STEP = (2400.0 - 600) / (2500 - 500) * 300 / 360 / 24;

    private double offset = 0;
    private double lastSet = 0;

    private static final double MIN_SENSOR_REFRESH_MS = 60;

    public final ServoWrapper extendoWrist;
    private final CRServo extendoIntake;
    private final ServoWrapper extendoLeft;
    private final ServoWrapper extendoRight;
    private final ColorSensor extendoColor;

    public Extendo(ServoWrapper extendoWrist, CRServo extendoIntake, ServoWrapper extendoLeft,
                   ServoWrapper extendoRight, ColorSensor extendoColor) {
        this.extendoWrist = extendoWrist;
        this.extendoIntake = extendoIntake;
        this.extendoLeft = extendoLeft;
        this.extendoRight = extendoRight;
        this.extendoColor = extendoColor;
    }

    public Extendo(HardwareMap hardwareMap) {
        this(
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "extendoPivot"), WRIST_DELAY),
                hardwareMap.get(CRServo.class, "intake"),
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "extendoLeft"), EXTENDO_DELAY),
                new ServoWrapper(hardwareMap.get(ServoImplEx.class, "extendoRight"), EXTENDO_DELAY),
                hardwareMap.get(ColorSensor.class, "extendoDist")
        );
    }

    public Task setWrist(double pos) {
        return extendoWrist.setPosition(pos + offset).with(new Task().oneshot(() -> lastSet = pos)).resources(this);
    }

    public Task setIntake(double power) {
        return new Task().oneshot(() -> extendoIntake.setPower(power)).resources(this);
    }

    public Task retract() {
        return extendoLeft.setPosition(LEFT_RETRACTED).with(extendoRight.setPosition(RIGHT_RETRACTED)).resources(this);
    }

    public Task extendPart(double frac) {
        Lerp left = new Lerp(new double[][]{{0, 1}, {LEFT_RETRACTED, LEFT_EXTENDED}});
        Lerp right = new Lerp(new double[][]{{0, 1}, {RIGHT_RETRACTED, RIGHT_EXTENDED}});
        frac = Math.min(1, Math.max(0, frac));
        return extendoLeft.setPosition(left.interpolate(frac))
                .with(extendoRight.setPosition(right.interpolate(frac)))
                .resources(this);
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

    private long lastTime = 0;
    private double lastDist = 0;
    private int lastAlpha = 0;
    private int lastRed = 0;
    private int lastGreen = 0;
    private int lastBlue = 0;

    private void updateSensor() {
        long current = System.currentTimeMillis();
        if (current - lastTime > MIN_SENSOR_REFRESH_MS) {
            lastTime = current;
            lastDist = ((DistanceSensor) extendoColor).getDistance(DistanceUnit.CM);
            int argb = extendoColor.argb();
            lastAlpha = (argb & (0xFF000000)) >>> (6 * 4);
            lastRed = (argb & (0x00FF0000)) >>> (4 * 4);
            lastGreen = (argb & (0x0000FF00)) >>> (2 * 4);
            lastBlue = (argb & (0x000000FF));
        }
    }

    public double getDistance() {
        updateSensor();
        return lastDist;
    }

    public boolean hasSample() {
        updateSensor();
        return lastDist < 1 && lastAlpha > 250;
    }

    public boolean hasCorrectSample() {
        if (!hasSample()) {
            return false;
        }
        updateSensor();
        int[] colors = {lastRed, lastGreen, lastBlue};
        System.out.printf("%d %d %d %d\n", lastAlpha, lastRed, lastGreen, lastBlue);
        if (colors[0] > Math.max(colors[1], colors[2]) * 1.5 && Color.currentColor == Color.RED) {
            System.out.println("red");
            return true;
        } else if (colors[2] > Math.max(colors[1], colors[0]) * 1.5 && Color.currentColor == Color.BLUE) {
            System.out.println("blue");
            return true;
        } else if (colors[1] > colors[0] && colors[1] > colors[2] * 2) {
            System.out.println("yellow");
            return true;
        }
        return false;
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