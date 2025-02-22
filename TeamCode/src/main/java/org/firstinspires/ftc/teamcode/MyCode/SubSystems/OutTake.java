package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;

public class OutTake {

    private CRServo outtake;  // CRServo used for outtake mechanism

    private final boolean IS_DEBUG;  // Flag for enabling/disabling debug mode

    private final double INTAKE_SPEED = 1;  // Power for intake
    private final double OUTTAKE_SPEED = -1;  // Power for outtake

    private final String SUBSYSTEM_NAME = "OutTake";  // Name of the subsystem for logging purposes

    private Telemetry telemetry;  // Telemetry object for displaying debug messages

    public OutTake(OpMode opMode, boolean isDebug) {
        outtake = opMode.hardwareMap.get(CRServo.class, "outTake");  // Get CRServo from hardware map

        this.telemetry = opMode.telemetry;  // Initialize telemetry
        this.IS_DEBUG = isDebug;  // Set the debug flag
    }

    public OutTake(OpMode opMode) {
        this(opMode, false);  // Default constructor without debug mode
    }

    // Initialize the outtake mechanism
    public void init() {
        outtake.setDirection(CRServo.Direction.FORWARD);  // Set the CRServo direction
        if (IS_DEBUG) {
            // Log debug message if debug mode is enabled
            DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Outtake initialized", true);
        }
    }

    // Set the power of the outtake mechanism
    public void setPower(double power) {
        outtake.setPower(power);
        if (IS_DEBUG) {
            // Log the power set to the outtake servo
            DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Set power: ", power);
        }
    }

    // Stop the outtake mechanism
    public void stop() {
        setPower(0);  // Set the power to 0 to stop the outtake
        if (IS_DEBUG) {
            // Log stop event
            DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Outtake stopped");
        }
    }

    // Activate outtake (set power to OUTTAKE_SPEED)
    public void outtake() {
        setPower(OUTTAKE_SPEED);
        if (IS_DEBUG) {
            // Log outtake event
            DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Outtake activated");
        }
    }

    // Activate intake (set power to INTAKE_SPEED)
    public void intake() {
        setPower(INTAKE_SPEED);
        if (IS_DEBUG) {
            // Log intake event
            DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Intake activated");
        }
    }
}
