package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;
import org.firstinspires.ftc.teamcode.TA2D2.PIDFController;
import org.firstinspires.ftc.teamcode.TA2D2.mathUtil.MathUtil;

public class Lift {

    private DcMotorEx leftLift, rightLift;

    private final boolean IS_DEBUG;

    private final double CPR = 0; // Encoder counts per revolution (adjust based on your motor specs)
    private final double SPOOL_DIAMETER = 0; // Diameter of the spool, used for distance conversion

    private final double levels[] = {0, 0, 0}; // Defined levels (could represent lift positions)

    private PIDFController pid = new PIDFController(0.11, 0.00015, 0.000000001, 0.4); // PID controller for lift control

    private final double TOLERANCE = 0; // Tolerance for PIDF calculations
    private final double AMP_LIMIT = 0; // Maximum allowed current draw before stopping the lift to avoid damage

    private double last_Power = 0; // Store the last power to check if the power change is within tolerance
    private double powerTolerance = 0.1; // Allowed tolerance for power change

    private final String SUBSYSTEM_NAME = "Lift"; // Subsystem name for logging

    private Telemetry telemetry;

    // Constructor with OpMode and debug flag
    public Lift(OpMode opMode, boolean isDebug) {
        leftLift = opMode.hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = opMode.hardwareMap.get(DcMotorEx.class, "rightLift");

        this.telemetry = opMode.telemetry;
        this.IS_DEBUG = isDebug;

        // Log if debugging is enabled
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Lift initialized");
    }

    // Constructor with default debug off
    public Lift(OpMode opMode) {
        this(opMode, false);
    }

    // Initialize motors and PID controller
    public void init() {
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        pid.setTolerance(TOLERANCE);

        resetEncoders();

        // Log initialization if debugging is enabled
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Lift initialized with zero power behavior and directions set");
    }

    // Reset encoder values
    public void resetEncoders() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Log encoder reset if debugging is enabled
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Encoders reset");
    }

    // Get the left lift position in distance units
    public double getLeftLiftPosition() {
        double position = MathUtil.convertTicksToDistance(CPR, SPOOL_DIAMETER, leftLift.getCurrentPosition());
        // Log lift position if debugging is enabled
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Left lift position", position);
        return position;
    }

    // Get the right lift position in distance units
    public double getRightLiftPosition() {
        double position = MathUtil.convertTicksToDistance(CPR, SPOOL_DIAMETER, rightLift.getCurrentPosition());
        // Log lift position if debugging is enabled
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Right lift position", position);
        return position;
    }

    // Get the average lift position from both sides
    public double getLiftPosition() {
        double position = (getLeftLiftPosition() + getRightLiftPosition()) / 2;
        // Log lift position if debugging is enabled
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Average lift position", position);
        return position;
    }

    // Get the current draw of the left motor in milliamps
    public double getAmpLeft() {
        double current = leftLift.getCurrent(CurrentUnit.MILLIAMPS);
        // Log current if debugging is enabled
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Left motor current", current);
        return current;
    }

    // Get the current draw of the right motor in milliamps
    public double getAmpRight() {
        double current = rightLift.getCurrent(CurrentUnit.MILLIAMPS);
        // Log current if debugging is enabled
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Right motor current", current);
        return current;
    }

    // Get the average current draw from both motors
    public double getAmp() {
        double current = (getAmpLeft() + getAmpRight()) / 2;
        // Log average current if debugging is enabled
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Average motor current", current);
        return current;
    }

    // Set the power of both motors
    public void setPower(double power) {

        // Check if the current exceeds the limit or if the power change is small
        if (getAmp() >= AMP_LIMIT && MathUtil.inTolerance(power, last_Power, powerTolerance)) {
            if (power < 0){
                resetEncoders();
            }
            power = 0; // Stop if overcurrent or power change is too small
        }

        last_Power = power; // Store last power value

        leftLift.setPower(power);
        rightLift.setPower(power);

        // Log power set action if debugging is enabled
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Set power", power);
    }

    // Stop the motors by setting power to 0
    public void stop() {
        setPower(0);
        // Log stop action if debugging is enabled
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Lift stopped");
    }

    // Set power using PID control to reach a specific position within a timeout
    public void setPowerWithPid(double pos, double timeOut) {
        pid.reset();
        pid.setSetPoint(pos);
        pid.setTolerance(timeOut);

        while (!pid.atSetPoint()) {
            setPower(pid.calculate(getLiftPosition()));
        }

        stop();

        // Log PID control action if debugging is enabled
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "PID control set power to reach position");
    }

    // Set power using PID control to reach a predefined level within a timeout
    public void setPowerWithPid(int level, double timeOut) {
        setPowerWithPid(levels[level], timeOut);

        // Log PID control for specific level if debugging is enabled
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "PID control set power for level " + level);
    }
}
