package org.firstinspires.ftc.teamcode.noncents;

public class PIDController {
    public double proportional;
    public double integral;
    // TODO smooth derivative calculations?
    public double derivative;
    public double integralCap;

    private boolean firstLoop = true;
    private long lastTime = 0;
    private double lastError = 0;
    private double errorAccum = 0;

    public PIDController(double proportional, double integral, double derivative) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.integralCap = Double.POSITIVE_INFINITY;
    }

    public PIDController(double proportional, double integral, double derivative, double integralCap) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.integralCap = integralCap;
    }

    public double update(double setpoint, double value, long time) {
        double error = setpoint - value;
        if (firstLoop) {
            lastTime = time;
            lastError = error;
            firstLoop = false;
        }
        long delta = time - lastTime;
        errorAccum += (error + lastError) / 2 * delta;
        errorAccum = Math.max(-integralCap, Math.min(integralCap, errorAccum));
        double p = proportional * error;
        double i = integral * errorAccum;
        double d = derivative * (error - lastError) / delta;
        return p + i + d;
    }
}
