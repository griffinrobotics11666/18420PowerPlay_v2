package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class HSVSliderMaskFunPipeline<HSV> extends OpenCvPipeline {
    Telemetry telemetry;
    public HSVSliderMaskFunPipeline (Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 255, 255);

    private Mat binaryMat = new Mat();
    private Mat maskForTheMat = new Mat();

    Mat HSV = new Mat();
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6,6));

    @Override
    public void init(Mat input) {
        // Executed before the first call to processFrame for subframes and other presistent frames.
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);


        telemetry.update();

        Core.inRange(HSV, lower, upper, binaryMat);
        Imgproc.erode(binaryMat, binaryMat, erodeElement);
        Imgproc.erode(binaryMat, binaryMat, erodeElement);
        Imgproc.dilate(binaryMat, binaryMat, dilateElement);
        Imgproc.dilate(binaryMat, binaryMat, dilateElement);

        maskForTheMat.release();


        Core.bitwise_and(input, input, maskForTheMat, binaryMat);

        //return HSV;
        return maskForTheMat;
        // Return the image that will be displayed in the viewport
        // (In this case the input mat directly)
    }


    @Override
    public void onViewportTapped() {
        // Executed when the image display is clicked by the mouse or tapped
        // This method is executed from the UI thread, so be careful to not
        // perform any sort heavy processing here! Your app might hang otherwise
    }

}