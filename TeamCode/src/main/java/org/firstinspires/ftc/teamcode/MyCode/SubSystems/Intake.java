package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;

public class Intake {

    private DcMotorSimple intake;  // Motor for the intake mechanism

    private final boolean IS_DEBUG;  // Flag for enabling/disabling debug mode

    private final double INTAKE_SPEED = 1;  // Speed for intake
    private final double OUTTAKE_SPEED = -1;  // Speed for outtake
    private final double AUTO_OUTTAKE_SPEED = -0.15;  // Speed for auto outtake

    private final String SUBSYSTEM_NAME = "Intake";  // Name of the subsystem for logging purposes

    private Telemetry telemetry;  // Telemetry object for displaying debug messages

    public Intake(OpMode opMode, boolean isDebug) {
        intake = opMode.hardwareMap.get(DcMotor.class, "intake");  // Get motor from hardware map

        this.telemetry = opMode.telemetry;  // Initialize telemetry
        this.IS_DEBUG = isDebug;  // Set the debug flag
        init();  // Initialize intake system
    }

    public Intake(OpMode opMode) {
        this(opMode, false);  // Default constructor without debug mode
    }

    // Initialize the intake mechanism
    public void init() {
        intake.setDirection(DcMotorSimple.Direction.FORWARD);  // Set motor direction to forward
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Intake initialized");  // Log debug message
    }

    // Set power to the intake motor
    public void setPower(double power) {
        intake.setPower(power);  // Set motor power
        DebugUtils.logDebug(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Set power", power);  // Log power setting
    }

    // Run intake with predefined speed
    public void intake() {
        setPower(INTAKE_SPEED);  // Set motor to intake speed
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Intake running");  // Log action
    }

    // Run outtake with predefined speed
    public void outtake() {
        setPower(OUTTAKE_SPEED);  // Set motor to outtake speed
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Outtake running");  // Log action
    }

    // Run auto outtake with predefined speed
    public void autoOuttake() {
        setPower(AUTO_OUTTAKE_SPEED);  // Set motor to auto outtake speed
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Auto Outtake running");  // Log action
    }

    // Stop the intake by setting power to zero
    public void stop() {
        setPower(0);  // Stop the motor
        DebugUtils.logDebugMessage(telemetry, IS_DEBUG, SUBSYSTEM_NAME, "Intake stopped");  // Log stop action
    }
}
