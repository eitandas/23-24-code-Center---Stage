package org.firstinspires.ftc.teamcode.MyCode.SubSystems;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

public  class ColorDetector implements VisionProcessor {
    Telemetry telemetry;
    Mat mat = new Mat();
    private static final boolean USE_WEBCAM = true;  // true for webcam, fal

    private boolean isRed = false;

    // se for phone camera
    //The variable to store our instance of the AprilTag processor.

    //The variable to store our instance of the ColorDetector Object Detection processor.

    //The variable to store our instance of the vision portal.
    // ??
    private VisionPortal myVisionPortal;
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    private Location location;

    static  Rect LEFT_ROI;
    static  Rect MIDDLE_ROI ;
    static  Rect RIGHT_ROI ;


    public ColorDetector(Telemetry telemetry, boolean isRed) {
        this.telemetry = telemetry;
        this.isRed = isRed;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        LEFT_ROI = new Rect(
                new Point(0, 40),
                new Point(width/3.0 -50, 408));

        MIDDLE_ROI = new Rect(
                new Point(width/3.0+50, 40),
                new Point(width*2/3.0 -50, 408));

        RIGHT_ROI = new Rect(
                new Point(width*2/3.0, 40),
                new Point(width, 408));

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV;
        Scalar hghHSV;
        if(isRed){
            lowHSV = new Scalar(0, 50, 70);
            hghHSV = new Scalar(10, 255, 255);
        }
        else{
            // working blue
            lowHSV = new Scalar(80,50, 70);
            hghHSV = new Scalar(120,255,255);


        }

        /*
        // working red
        Scalar
        Scalar
         */

        Core.inRange(frame, lowHSV, hghHSV, frame);
        Mat left = frame.submat(LEFT_ROI);
        Mat right = frame.submat(RIGHT_ROI);
        Mat middle = frame.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        right.release();
        middle.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("middle percentage", Math.round(middleValue * 100) + "%");


        boolean Left = leftValue > rightValue && leftValue > middleValue;
        boolean Right = rightValue > leftValue && rightValue > middleValue;
        boolean Middle = middleValue > rightValue && middleValue > leftValue;

        if (Middle) {
            location = Location.MIDDLE;
            telemetry.addData("TeamProp Location", "middle");

        } else if (Right) {
            location = Location.RIGHT;
            telemetry.addData("TeamProp Location", "right");
        } else {
            location = Location.LEFT;
            if(Left)
            {
                telemetry.addData("TeamProp Location", "left");
            }
            else{
                telemetry.addData("TeamProp Location", "not found");
            }
        }
        telemetry.update();



        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_GRAY2RGB);

        Scalar Acolor_cub = new Scalar(0, 255, 0);
        Scalar Bcolor_cub = new Scalar(255, 0, 0);

        Imgproc.rectangle(frame, LEFT_ROI, location == Location.LEFT ? Acolor_cub : Bcolor_cub,10);
        Imgproc.rectangle(frame, RIGHT_ROI, location == Location.RIGHT ? Acolor_cub : Bcolor_cub,10);
        Imgproc.rectangle(frame, MIDDLE_ROI, location == Location.MIDDLE ? Acolor_cub : Bcolor_cub,10);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    // public static ColorDetector easyCreateWithDefaults()
    //   {
    //      return new ColorDetector.Builder().build();
    //  }
    // public static class Builder{

    // }







    public void setRed(boolean isRed){
        this.isRed = isRed;
    }

    // @Override



    public Location getLocation() {
        return location;


    }



}