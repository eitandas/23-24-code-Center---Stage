package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import android.app.ApplicationErrorReport;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private Servo shooter;

    private final boolean IS_DEBUG;

    public Shooter(OpMode opMode, boolean isDebug) {
        shooter = opMode.hardwareMap.get(Servo.class, "shooter");

        this.IS_DEBUG = isDebug;
    }

    public Shooter(OpMode opMode) {
        this(opMode, false);
    }

    public void init() {
        shooter.setDirection(Servo.Direction.FORWARD);
    }

    public void shoot(){
        shooter.setPosition(1);
    }

    public void stop(){
        shooter.setPosition(0);
    }


}
