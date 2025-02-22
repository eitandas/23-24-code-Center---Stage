package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TA2D2.mathUtil.MathUtil;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;

/**
 * The Arm class controls a dual-servo arm system with position feedback from analog sensors.
 * It allows setting the arm position manually or by predefined levels.
 */
public class Arm {

    // Servos controlling the arm movement
    private Servo armLeft, armRight;

    // Analog sensors providing feedback on arm position
    private AnalogInput armLeftSensor, armRightSensor;

    // Debug mode flag
    private final boolean IS_DEBUG;

    // Predefined arm levels for quick positioning
    private double levels[] = {0, 0.55, 1};

    // Name of the subsystem for debugging
    private final String SUBSYSTEM_NAME = "Arm";

    // Telemetry instance for logging
    private Telemetry telemetry;

    private final double LEFT_ARM_OFFSET = 0.0;  // Adjust as needed
    private final double RIGHT_ARM_OFFSET = 0.0; // Adjust as needed

    /**
     * Constructs an Arm instance, initializing hardware components.
     *
     * @param opMode   The active OpMode providing hardware mapping and telemetry.
     * @param isDebug  Enables or disables debug logging.
     */
    public Arm(OpMode opMode, boolean isDebug) {
        // Initialize servos from hardware map
        armLeft = opMode.hardwareMap.get(Servo.class, "armLeft");
        armRight = opMode.hardwareMap.get(Servo.class, "armRight");

        // Initialize analog sensors from hardware map
        armLeftSensor = opMode.hardwareMap.get(AnalogInput.class, "armLeftSensor");
        armRightSensor = opMode.hardwareMap.get(AnalogInput.class, "armRightSensor");

        // Assign telemetry instance
        this.telemetry = opMode.telemetry;

        // Store debug mode flag
        this.IS_DEBUG = isDebug;

        // Log that the arm subsystem has been initialized
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Arm constructor complete");
    }

    /**
     * Constructs an Arm instance with debug mode disabled by default.
     *
     * @param opMode The active OpMode providing hardware mapping and telemetry.
     */
    public Arm(OpMode opMode) {
        this(opMode, false);
    }

    /**
     * Initializes the arm subsystem, setting servo directions.
     */
    public void init() {
        // Set servo directions to ensure they move in the correct orientation
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight.setDirection(Servo.Direction.FORWARD);

        // Log that the initialization is complete
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "init complete");
    }

    /**
     * Gets the current position of the left arm in degrees.
     *
     * @return The left arm position in degrees.
     */
    public double getLeftArmPosition() {
        // Convert sensor voltage to degrees using MathUtil
        double leftPos = MathUtil.voltageToDegrees(armLeftSensor.getVoltage()) + LEFT_ARM_OFFSET;

        // Log the left arm position
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "LeftArmPosition", leftPos);

        return leftPos;
    }

    /**
     * Gets the current position of the right arm in degrees.
     *
     * @return The right arm position in degrees.
     */
    public double getRightArmPosition() {
        // Convert sensor voltage to degrees using MathUtil
        double rightPos = MathUtil.voltageToDegrees(armRightSensor.getVoltage()) + RIGHT_ARM_OFFSET;

        // Log the right arm position
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "RightArmPosition", rightPos);

        return rightPos;
    }

    /**
     * Computes the average position of both arms.
     *
     * @return The average arm position in degrees.
     */
    public double getArmPosition() {
        double leftPos = getLeftArmPosition();
        double rightPos = getRightArmPosition();
        double avgPos = (leftPos + rightPos) / 2;

        // Log the average arm position
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "AverageArmPosition", avgPos);

        return avgPos;
    }

    /**
     * Sets the arm position by directly specifying a servo position value.
     *
     * @param position The target position value (0.0 to 1.0).
     */
    public void setPosition(double position) {
        // Move both servos to the specified position
        armLeft.setPosition(position);
        armRight.setPosition(position);

        // Log the set position
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "SetPosition", position);
    }

    /**
     * Sets the arm position based on a predefined level index.
     *
     * @param level The index of the level (0, 1, or 2).
     */
    public void setPosition(int level) {
        if (level < 0 || level >= levels.length) {
            DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Invalid level: " + level);
            return;
        }
        setPosition(levels[level]);
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Set position by level: " + level);
    }
}
