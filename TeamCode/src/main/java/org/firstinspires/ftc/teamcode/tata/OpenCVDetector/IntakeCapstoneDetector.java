package org.firstinspires.ftc.teamcode.tata.OpenCVDetector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class IntakeCapstoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum IntakeStatus {
        EMPTY,
        FULL
    }
    private IntakeStatus intake_status = IntakeStatus.EMPTY;

    static final Rect CAP_ROI = new Rect(
            new Point(220, 140),
            new Point(420, 340));
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public IntakeCapstoneDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(15, 30, 50);
        Scalar highHSV = new Scalar(42, 255, 255);

        //Scalar lowHSV = new Scalar(200, 10, 20);
        //Scalar highHSV = new Scalar(255, 100, 100);
        //ducks above
        //CHANGE THIS ONCE U RUN CODE

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(CAP_ROI);

        double leftValue = Core.sumElems(left).val[0] / CAP_ROI.area() / 255;

        left.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");

        boolean capLeft = leftValue > PERCENT_COLOR_THRESHOLD;

        if (capLeft) {
            intake_status = IntakeStatus.FULL;
            telemetry.addData("Capstone Location", "FULL");
        }
        else {
            intake_status = IntakeStatus.EMPTY;
            telemetry.addData("Capstone Location", "EMPTY");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNotCap = new Scalar(255, 0, 0);
        Scalar colorCap = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, CAP_ROI, intake_status == IntakeStatus.FULL? colorCap:colorNotCap);

        return mat;
    }

    public IntakeStatus getIntakeStatus() {
        return intake_status;
    }
}