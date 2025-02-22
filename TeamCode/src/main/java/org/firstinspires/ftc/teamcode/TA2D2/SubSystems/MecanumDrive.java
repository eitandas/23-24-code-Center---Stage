package org.firstinspires.ftc.teamcode.TA2D2.SubSystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;
import org.firstinspires.ftc.teamcode.TA2D2.PIDFController;
import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.TA2D2.mathUtil.MathUtil;

public class MecanumDrive {
    // Motors for the Mecanum drive system
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Encoders for position tracking
    private final DcMotor leftXEncoder, rightXEncoder, yEncoder;

    // IMU for yaw angle (robot's heading)
    private IMU imu;

    // OpMode (telemetry) and debug mode flag
    private final OpMode opMode;
    private final boolean isDebugMode;

    // Pose variables for the robot's position and orientation
    private Pose2d rotatedPos = new Pose2d(0, 0, 0);
    private Pose2d unRotatedPos = new Pose2d(0, 0, 0);
    private double angle = 0;

    // PID controllers for each axis
    private PIDFController xPid = new PIDFController(0.000892, 0.0001, 0.000001, 0);
    private PIDFController yPid = new PIDFController(0.0023, 0.0002, 0, 0);
    private PIDFController zPid = new PIDFController(0.65, 0.000000001, 0, 0);

    // Tolerances for PID control
    private final double xTolerance = 10;
    private final double yTolerance = 10;
    private final double zTolerance = 2;

    private double xMMPerTick = (8192.0 * 1.0)/ (35 * Math.PI);
    private double yMMPerTick = (8192.0 * 1.0)/ (35 * Math.PI);


    // Name for this subsystem (used in debug logs)
    private static final String SUBSYSTEM_NAME = "Mecanum Drive";

    // Constructor for the MecanumDrive class
    public MecanumDrive(OpMode opMode, boolean isDebug) {
        // Initialize motors (front-left, front-right, back-left, back-right)
        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");

        // Initialize encoders (for position tracking)
        leftXEncoder = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        rightXEncoder = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        yEncoder = opMode.hardwareMap.get(DcMotor.class, "backRight");

        // Initialize the IMU (for yaw angle)
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        // Set debug mode flag
        this.isDebugMode = isDebug;

        //set the opMode
        this.opMode = opMode;

        // Initialize the drive system
        init();
    }

    /**
     * Initializes the Mecanum drive system by setting the motors' behaviors and tolerances for PID controllers.
     */
    public void init() {
        // Set motors to BRAKE mode when power is zero (stopping the robot)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID tolerances
        xPid.setTolerance(xTolerance);
        yPid.setTolerance(yTolerance);
        zPid.setTolerance(zTolerance);

        //
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
    }

    /**
     * Resets all encoders to zero and sets them to run without encoder tracking.
     */
    public void resetEncoders() {
        leftXEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightXEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set encoders to RUN_WITHOUT_ENCODER mode to allow for position tracking
        leftXEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightXEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Updates the robot's position using encoder values and IMU (yaw) angle.
     */
    public void update() {
        // Get the current yaw (heading) of the robot from the IMU
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Normalize the angle once to ensure it is within -180 to 180 degrees
        angle = MathUtil.normalizeAngle(angle);

        // Calculate the rotated position using the encoder values
        rotatedPos = new Pose2d(xMMPerTick *
                (leftXEncoder.getCurrentPosition() + rightXEncoder.getCurrentPosition()) / 2,
                yMMPerTick * yEncoder.getCurrentPosition(),
                angle
        );

        // Rotate the position based on the current angle of the robot
        rotatedPos.rotateByDegrees(angle);

        // Debug logging for position and angle
        DebugUtils.logDebug(opMode.telemetry, isDebugMode, SUBSYSTEM_NAME, "Rotated Position", rotatedPos);
        DebugUtils.logDebug(opMode.telemetry, isDebugMode, SUBSYSTEM_NAME, "Angle", angle);
    }

    /**
     * Sets the power for each motor based on the target pose (x, y, z) of the robot.
     * @param pose The target pose for the robot (x, y, z)
     */
    public void setPower(Pose2d pose) {
        // Get the x, y, and z values from the pose
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getAngle();

        // Denominator used for normalizing motor powers based on input values
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1);

        // Calculate individual motor powers based on the mecanum drive formula
        double rightFrontPower = (y - x - z) / denominator;
        double rightBackPower = (y + x - z) / denominator;
        double leftFrontPower = (y + x + z) / denominator;
        double leftBackPower = (y - x + z) / denominator;

        // Set the calculated power to the motors
        frontLeft.setPower(leftFrontPower);
        backLeft.setPower(leftBackPower);
        frontRight.setPower(rightFrontPower);
        backRight.setPower(rightBackPower);

        // Debug logging for motor powers
        DebugUtils.logDebug(opMode.telemetry, isDebugMode, SUBSYSTEM_NAME, "Left Front Power", leftFrontPower);
        DebugUtils.logDebug(opMode.telemetry, isDebugMode, SUBSYSTEM_NAME, "Right Front Power", rightFrontPower);
        DebugUtils.logDebug(opMode.telemetry, isDebugMode, SUBSYSTEM_NAME, "Left Back Power", leftBackPower);
        DebugUtils.logDebug(opMode.telemetry, isDebugMode, SUBSYSTEM_NAME, "Right Back Power", rightBackPower);
    }

    /**
     * Stops the robot by setting all motor powers to zero.
     */
    public void stop() {
        setPower(new Pose2d(0, 0, 0));
    }

    /**
     * Drives the robot using field-relative control. The pose is rotated based on the robot's current heading.
     * @param pose The target pose for the robot, in field-relative coordinates.
     */
    public void fieldDrive(Pose2d pose) {
        // Rotate the pose by the robot's current yaw angle
        pose.rotateByDegrees(angle);

        // Set motor powers based on the rotated pose
        setPower(pose);

        // Debug logging for field drive execution
        DebugUtils.logDebugMessage(opMode.telemetry, isDebugMode, SUBSYSTEM_NAME, "Field drive executed.");
    }

    /**
     * Drives the robot using PID control to reach the target pose (x, y, z).
     * @param pose The target pose for the robot.
     * @param timeOut The time limit for the PID control loop.
     */
    public void pidDrive(Pose2d pose, double timeOut) {
        // Set the timeout for each PID controller
        xPid.setTimeout(timeOut);
        yPid.setTimeout(timeOut);
        zPid.setTimeout(timeOut);

        // Rotate the target pose by the current angle of the robot and normalize it
        pose.rotateByDegrees(angle);

        // Set the setpoints for the PID controllers
        xPid.setSetPoint(pose.getX());
        yPid.setSetPoint(pose.getY());
        zPid.setSetPoint(pose.getAngle());

        // Reset the PID controllers
        xPid.reset();
        yPid.reset();
        zPid.reset();

        // Run the PID control loop until the robot reaches the setpoints
        while (!xPid.atSetPoint() || !yPid.atSetPoint() || !zPid.atSetPoint()) {
            // Calculate the motor powers based on the PID outputs
            double xPower = xPid.calculate(rotatedPos.getX());
            double yPower = yPid.calculate(rotatedPos.getY());
            double zPower = zPid.calculate(rotatedPos.getAngle());

            // Set the motor powers to reach the target pose
            setPower(new Pose2d(yPower, xPower, zPower));
        }

        // Stop the robot once the target pose is reached
        stop();
    }

    public double getAngle(){
        return this.angle;
    }
}
