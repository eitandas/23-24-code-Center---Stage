package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class OutTake {

    private CRServo outtake;

    private final boolean IS_DEBUG;

    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = -1;

    public OutTake(OpMode opMode, boolean isDebug) {
        outtake = opMode.hardwareMap.get(CRServo.class, "outTake");
        this.IS_DEBUG = isDebug;
    }

    public OutTake(OpMode opMode) {
        this(opMode, false);
    }

    public void init() {
        outtake.setDirection(CRServo.Direction.FORWARD);
    }

    public void setPower(double power) {
        outtake.setPower(power);
    }

    public void stop() {
        setPower(0);
    }

    public void outtake() {
        setPower(OUTTAKE_SPEED);
    }

    public void intake() {
        setPower(INTAKE_SPEED);
    }

}
