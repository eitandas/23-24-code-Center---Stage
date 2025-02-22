package org.firstinspires.ftc.teamcode.TA2D2.Pathing;

import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.TA2D2.SubSystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.TA2D2.mathUtil.MathUtil;

/**
 * A class to follow paths using splines, with support for strafing, slant (diagonal) movement, and speed control.
 */
public class PathFollower {
    private final MecanumDrive drive; // Mecanum drive system
    private final Spline xSpline, ySpline; // Splines for x and y coordinates
    private double t = 0; // Parameter for the spline (0 <= t <= 1)
    private boolean isStrafing = false; // Strafe mode flag
    private boolean isSlanting = false; // Slant (diagonal) mode flag
    private double speed = 1.0; // Default speed (0 to 1)

    /**
     * Constructs a PathFollower.
     *
     * @param drive The MecanumDrive instance.
     * @param start The starting pose of the path.
     * @param end The ending pose of the path.
     */
    public PathFollower(MecanumDrive drive, Pose2d start, Pose2d end) {
        this.drive = drive;

        // Create splines for x and y coordinates
        this.xSpline = new Spline(start.getX(), end.getX(), start.getAngle(), end.getAngle());
        this.ySpline = new Spline(start.getY(), end.getY(), start.getAngle(), end.getAngle());
    }

    /**
     * Enables or disables strafing mode.
     *
     * @param isStrafing Whether strafing is enabled.
     */
    public void setStrafing(boolean isStrafing) {
        this.isStrafing = isStrafing;
    }

    /**
     * Enables or disables slant (diagonal) mode.
     *
     * @param isSlanting Whether slanting is enabled.
     */
    public void setSlanting(boolean isSlanting) {
        this.isSlanting = isSlanting;
    }

    /**
     * Sets the speed of the robot (0 to 1).
     *
     * @param speed The desired speed.
     */
    public void setSpeed(double speed) {
        this.speed = MathUtil.clamp(speed, 0, 1);
    }

    /**
     * Follows the path.
     *
     * @param deltaT The time step for updating the path.
     */
    public void followPath(double deltaT) {
        if (t >= 1.0) {
            // Stop the robot if the path is complete
            drive.stop();
            return;
        }

        // Calculate target position using spline evaluations
        double targetX = xSpline.evaluate(t);
        double targetY = ySpline.evaluate(t);
        double targetAngle = calculateTargetAngle(t);

        // Create a target pose
        Pose2d targetPose = new Pose2d(targetX, targetY, Math.toDegrees(targetAngle));

        // Adjust for strafing mode
        if (isStrafing) {
            // Maintain the current angle while strafing
            targetPose.setAngle(drive.getAngle());
        }

        // Adjust for slant mode
        if (isSlanting) {
            // Combine forward/backward and strafing motion for diagonal movement
            double forwardPower = targetPose.getY();
            double strafePower = targetPose.getX();
            targetPose = new Pose2d(strafePower, forwardPower, targetPose.getAngle());
        }

        // Scale the target pose by the speed
        targetPose = new Pose2d(
                targetPose.getX() * speed,
                targetPose.getY() * speed,
                targetPose.getAngle() * speed
        );

        // Use the MecanumDrive to move to the target pose
        drive.pidDrive(targetPose, 0.1); // Adjust timeout as needed

        // Increment the parameter t based on time elapsed
        t += deltaT * speed; // Adjust speed impact on time step
    }

    /**
     * Calculates the target angle from the spline derivatives.
     *
     * @param t The current parameter value.
     * @return The calculated target angle.
     */
    private double calculateTargetAngle(double t) {
        double deltaX = xSpline.evaluateDerivative(t);
        double deltaY = ySpline.evaluateDerivative(t);
        return Math.atan2(deltaY, deltaX);
    }

    /**
     * Resets the path follower.
     */
    public void reset() {
        t = 0;
    }

    /**
     * Checks if the path is complete.
     *
     * @return True if the path is complete.
     */
    public boolean isPathComplete() {
        return t >= 1.0;
    }
}
