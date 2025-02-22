package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TA2D2.mathUtil.MathUtil;

public class Arm {

    private Servo armLeft, armRight;

    private AnalogInput armLeftSensor, armRightSensor;

    private final boolean IS_DEBUG;

    private double levels[] = {0, 0.55 , 1};


    public Arm(OpMode opMode, boolean isDebug) {
        armLeft = opMode.hardwareMap.get(Servo.class, "armLeft");
        armRight = opMode.hardwareMap.get(Servo.class, "armRight");

        armLeftSensor = opMode.hardwareMap.get(AnalogInput.class, "armLeftSensor");
        armRightSensor = opMode.hardwareMap.get(AnalogInput.class, "armRightSensor");

        this.IS_DEBUG = isDebug;

    }

    public Arm(OpMode opMode) {
        this(opMode, false);
    }

    public void init(){
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight.setDirection(Servo.Direction.FORWARD);
    }

    public double getLeftArmPosition() {
        return MathUtil.voltageToDegrees(armLeftSensor.getVoltage());
    }

    public double getRightArmPosition() {
        return MathUtil.voltageToDegrees(armRightSensor.getVoltage());
    }

    public double getArmPosition() {
        return (getLeftArmPosition() + getRightArmPosition()) / 2;
    }

    public void setPosition(double position) {
        armLeft.setPosition(position);
        armRight.setPosition(position);
    }

    public void setPosition(int level){
        setPosition(levels[level]);
    }

}
