package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private final DcMotorEx[] wheels;

    public Drivetrain(DcMotorEx[] wheels) {
        for (DcMotor wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        wheels[0].setDirection(DcMotor.Direction.REVERSE);
        wheels[1].setDirection(DcMotor.Direction.REVERSE);
        wheels[2].setDirection(DcMotor.Direction.REVERSE);
        wheels[3].setDirection(DcMotor.Direction.REVERSE);
        this.wheels = wheels;
    }

    public Drivetrain(HardwareMap hardwareMap) {
        this(new DcMotorEx[]{
                hardwareMap.get(DcMotorEx.class, "wheelFrontLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelFrontRight"),
                hardwareMap.get(DcMotorEx.class, "wheelBackLeft"),
                hardwareMap.get(DcMotorEx.class, "wheelBackRight")
        });
    }

    public double[] mix(double forward, double lateral, double rotate) {
        Lerp forwardLerp = new Lerp(new double[][] {
                {0, 0},
                {0.02, 0.08},
                {0.5, 0.35},
                {1, 1}
        });
        Lerp lateralLerp = forwardLerp;
        Lerp rotateLerp = new Lerp(new double[][] {
                {0, 0},
                {0.02, 0.08},
                {0.7, 0.35},
                {1, 1}
        });;
        forward = forwardLerp.interpolateMagnitude(forward);
        lateral = lateralLerp.interpolateMagnitude(lateral);
        rotate = rotateLerp.interpolateMagnitude(rotate);
        return new double[]{
                forward + lateral + rotate,
                forward - lateral - rotate,
                forward - lateral + rotate,
                forward + lateral - rotate
        };
    }

    public void setPowers(double[] powers) {
        double max = 1;
        for (int i = 0; i < wheels.length; i++) {
            max = Math.max(Math.abs(powers[i]), max);
        }
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setPower(powers[i] / max);
        }
    }
}
