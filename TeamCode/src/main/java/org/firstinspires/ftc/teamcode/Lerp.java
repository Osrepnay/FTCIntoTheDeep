package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.util.Comparator;

public class Lerp {
    private final double[][] points;

    public Lerp(double[][] points) {
        this.points = Arrays.stream(points).sorted(Comparator.comparing(p -> p[0])).toArray(double[][]::new);
    }

    public double interpolate(double x) {
        int i = 0;
        while (i < points.length && points[i][0] < x) {
            i++;
        }
        i = Math.min(points.length - 1, Math.max(1, i));
        return points[i - 1][1] + (points[i][1] - points[i - 1][1]) * (x - points[i - 1][0]) / (points[i][0] - points[i - 1][0]);
    }

    public double interpolateMagnitude(double x) {
        return Math.signum(x) * interpolate(Math.abs(x));
    }
}
