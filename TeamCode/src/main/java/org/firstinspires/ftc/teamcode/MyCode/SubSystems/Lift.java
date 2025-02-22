package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.TA2D2.PIDFController;
import org.firstinspires.ftc.teamcode.TA2D2.mathUtil.MathUtil;

public class Lift {

    private DcMotorEx leftLift, rightLift;

    private final boolean IS_DEBUG;

    private final double CPR = 0;

    private final double SPOOL_DIAMETER = 0;

    private final double levels[] = {0, 0, 0};

    private PIDFController pid = new PIDFController(0.11, 0.00015, 0.000000001, 0.4);

    private final double TOLERANCE = 0;

    private final double AMP_LIMIT = 0;

    private double last_Power = 0;

    private double powerTolerance = 0.1;


    public Lift(OpMode opMode, boolean isDebug) {
        leftLift = opMode.hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = opMode.hardwareMap.get(DcMotorEx.class, "rightLift");

        this.IS_DEBUG = isDebug;
    }

    public Lift(OpMode opMode) {
        this(opMode, false);
    }

    public void init() {
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        pid.setTolerance(TOLERANCE);

        resetEncoders();
    }

    public void resetEncoders() {
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getLeftLiftPosition() {
        return MathUtil.convertTicksToDistance(CPR, SPOOL_DIAMETER, leftLift.getCurrentPosition());
    }

    public double getRightLiftPosition() {
        return MathUtil.convertTicksToDistance(CPR, SPOOL_DIAMETER, rightLift.getCurrentPosition());
    }

    public double getLiftPosition() {
        return (getLeftLiftPosition() + getRightLiftPosition()) / 2;
    }

    public double getAmpLeft () {
        return leftLift.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getAmpRight () {
        return rightLift.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getAmp () {
        return (getAmpLeft() + getAmpRight()) / 2;
    }

    public void setPower(double power) {

        if (getAmp() >= AMP_LIMIT && MathUtil.inTolerance(power, last_Power, powerTolerance)) {
            if (power < 0){
                resetEncoders();
            }
            power = 0;
        }

        last_Power = power;

        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void stop() {
        setPower(0);
    }

    public void setPowerWithPid(double pos, double timeOut) {
        pid.reset();
        pid.setSetPoint(pos);
        pid.setTolerance(timeOut);

        while (!pid.atSetPoint()){
            setPower(pid.calculate(getLiftPosition()));
        }

        stop();
    }

    public void setPowerWithPid(int level, double timeOut) {
        setPowerWithPid(levels[level], timeOut);
    }
}
