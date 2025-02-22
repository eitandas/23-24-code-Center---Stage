package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {

    private DcMotorSimple intake;

    private final boolean IS_DEBUG;

    private final double INTAKE_SPEED = 1;

    private final double OUTTAKE_SPEED = -1;

    private final double AUTO_OUTTAKE_SPEED = -0.15;

    public Intake(OpMode opMode, boolean isDebug) {
        this.intake = opMode.hardwareMap.get(DcMotor.class, "intake");

        this.IS_DEBUG = isDebug;

        init();
    }

    public void init(){
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void intake(){
        setPower(INTAKE_SPEED);
    }

    public void outtake(){
        setPower(OUTTAKE_SPEED);
    }

    public void autoOuttake(){
        setPower(AUTO_OUTTAKE_SPEED);
    }

    public void stop(){
        setPower(0);
    }


}
