package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;

public class Shooter {

    private Servo shooter;  // Servo used for shooter mechanism

    private final boolean IS_DEBUG;  // Flag for enabling/disabling debug mode

    private final String SUBSYSTEM_NAME = "Shooter";  // Name of the subsystem for logging purposes

    private Telemetry telemetry;  // Telemetry object for displaying debug messages

    public Shooter(OpMode opMode, boolean isDebug) {
        shooter = opMode.hardwareMap.get(Servo.class, "shooter");  // Get Servo from hardware map

        this.telemetry = opMode.telemetry;  // Initialize telemetry
        this.IS_DEBUG = isDebug;  // Set the debug flag
    }

    public Shooter(OpMode opMode) {
        this(opMode, false);  // Default constructor without debug mode
    }

    // Initialize the shooter mechanism
    public void init() {
        shooter.setDirection(Servo.Direction.FORWARD);  // Set the servo direction to forward
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Shooter initialized");  // Log debug message when the shooter is initialized
    }

    // Activate the shooter (set servo to 1 for shooting)
    public void shoot() {
        shooter.setPosition(1);  // Move servo to full forward position to shoot
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Shooter activated");  // Log debug message when shooting is activated
    }

    // Stop the shooter (set servo to 0)
    public void stop() {
        shooter.setPosition(0);  // Move servo to 0 to stop the shooter
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Shooter stopped");  // Log debug message when shooter is stopped
    }
}
