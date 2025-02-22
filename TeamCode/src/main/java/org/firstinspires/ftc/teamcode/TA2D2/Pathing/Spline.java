package org.firstinspires.ftc.teamcode.TA2D2.Pathing;

/**
 * Represents a cubic spline segment.
 */
public class Spline {
    private final double a, b, c, d; // Coefficients for the cubic spline

    /**
     * Constructs a cubic spline segment.
     *
     * @param p0 The starting point of the spline.
     * @param p1 The ending point of the spline.
     * @param m0 The slope at the starting point.
     * @param m1 The slope at the ending point.
     */
    public Spline(double p0, double p1, double m0, double m1) {
        // Coefficients for the cubic spline equation: f(t) = a*t^3 + b*t^2 + c*t + d
        this.a = 2 * p0 - 2 * p1 + m0 + m1;
        this.b = -3 * p0 + 3 * p1 - 2 * m0 - m1;
        this.c = m0;
        this.d = p0;
    }

    /**
     * Evaluates the spline at a given parameter t.
     *
     * @param t The parameter (0 <= t <= 1).
     * @return The value of the spline at t.
     */
    public double evaluate(double t) {
        return a * t * t * t + b * t * t + c * t + d;
    }

    /**
     * Evaluates the derivative of the spline at a given parameter t.
     *
     * @param t The parameter (0 <= t <= 1).
     * @return The derivative of the spline at t.
     */
    public double evaluateDerivative(double t) {
        return 3 * a * t * t + 2 * b * t + c;
    }
}