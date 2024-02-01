package org.firstinspires.ftc.teamcode.code;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ParkingPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    boolean isBlue;
    public ParkingPipeline(Telemetry t, boolean isBlue){
        telemetry = t;
        this.isBlue = isBlue;
    }

    position cpos;


    Point i0 = new Point(0,0), i1 = new Point(426,720), i2 = new Point(426,0), i3 = new Point(852,720), i4 = new Point(852,0), i5 = new Point(1278,720);


    Rect leftPos = new Rect(
      i0,
      i1
    );
    Rect middlePos = new Rect(
            i2,
            i3
    );
    Rect rightPos = new Rect(
            i4,
            i5
    );

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        Scalar lowVal, highVal;
        if(isBlue){
            lowVal = new Scalar(RobotConstants.blue_hue_low, RobotConstants.blue_sat_low, RobotConstants.blue_val_low);
            highVal = new Scalar(RobotConstants.blue_hue_high, RobotConstants.blue_sat_high, RobotConstants.blue_val_high);
        }else{
            lowVal = new Scalar(RobotConstants.red_hue_low, RobotConstants.red_sat_low, RobotConstants.red_val_low);
            highVal = new Scalar(RobotConstants.red_hue_high, RobotConstants.red_sat_high, RobotConstants.red_val_high);
        }

        Mat mainMat = new Mat();
        Mat leftMat, middleMat, rightMat;

        Core.inRange(input, lowVal, highVal, mainMat);

        leftMat = mainMat.submat(leftPos);
        middleMat = mainMat.submat(middlePos);
        rightMat = mainMat.submat(rightPos);

        double left = Core.sumElems(leftMat).val[0];
        double middle = Core.sumElems(middleMat).val[0];
        double right = Core.sumElems(rightMat).val[0];

        telemetry.addData("Left: ", left);
        telemetry.addData("Middle: ", middle);
        telemetry.addData("Right: ", right);

        if(left > middle && left > right){
            //left
            telemetry.addData("Sees", "Left");
            Imgproc.rectangle(input, i0,i1, new Scalar(0,255,0),10);
            Imgproc.rectangle(input, i2,i3, new Scalar(0,0,0),10);
            Imgproc.rectangle(input, i4,i5, new Scalar(0,0,0),10);
            this.cpos = position.left;
        }else if(middle > right){
            //middle
            telemetry.addData("Sees", "Middle");
            Imgproc.rectangle(input, i2,i3, new Scalar(0,255,0),10);
            Imgproc.rectangle(input, i0,i1, new Scalar(0,0,0),10);
            Imgproc.rectangle(input, i4,i5, new Scalar(0,0,0),10);
            this.cpos = position.center;
        }else{
            //right
            telemetry.addData("Sees", "Right");
            Imgproc.rectangle(input, i4,i5, new Scalar(0,255,0),10);
            Imgproc.rectangle(input, i0,i1, new Scalar(0,0,0),10);
            Imgproc.rectangle(input, i2,i3, new Scalar(0,0,0),10);
            this.cpos = position.right;
        }
        telemetry.update();


        return mainMat;
    }

    public position getPos(){
        return cpos;
    }

    public enum position{
        left,
        right,
        center
    }
}
