package org.firstinspires.ftc.teamcode.MyCode.OpModes.Autonomouses;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MyCode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.MyCode.SubSystems.ColorDetector;
import org.firstinspires.ftc.teamcode.MyCode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.MyCode.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.MyCode.SubSystems.OutTake;
import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.TA2D2.SubSystems.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;


@TeleOp
public class BlueFar extends LinearOpMode {

    private VisionPortal visionPortal;

    private ColorDetector.Location location;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(this, false);
        Lift lift = new Lift(this, false);
        Intake intake = new Intake(this, false);
        OutTake outtake = new OutTake(this, false);
        Arm arm = new Arm(this, false);

        ColorDetector detector = new ColorDetector(telemetry, false);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(800, 448));
        builder.enableLiveView(true);
        builder.addProcessor(detector);
        visionPortal = builder.build();
        while (opModeInInit()){
            location = detector.getLocation();
            sleep(30);
        }
        visionPortal.setProcessorEnabled(detector, false);
        waitForStart();
        if (opModeIsActive()){
            if (location == ColorDetector.Location.LEFT) {
                drive.pidDrive(new Pose2d(500, -300, 0), 1500);
                intake.autoOuttake();
                sleep(2000);
                intake.stop();
                drive.pidDrive(new Pose2d(100, -260, 0), 1500);
                drive.pidDrive(new Pose2d(0, -20, 0), 2500);
                drive.pidDrive(new Pose2d(520, -620, 0), 3000);
                drive.pidDrive(new Pose2d(520, -620, 90), 2500);
                lift.setPowerWithPid(1, 3000);
                arm.setPosition(1);
                drive.pidDrive(new Pose2d(540,-880,100), 2000);
                outtake.outtake();
                drive.pidDrive(new Pose2d(500, -680, 100), 2500);
                arm.setPosition(0);
                outtake.stop();
                lift.setPowerWithPid(0, 3000);
            }
        }
    }
}
