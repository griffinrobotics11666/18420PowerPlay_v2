package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

public class WherePipeline extends OpenCvPipeline {
    //TODO - 1 Use Threshold to determine pixels that contain cones
    //TODO - 2 Use Threshold image to determine center point
    //TODO - 3 Use Threshold image and smooth it?
    //TODO - 4 Use Threshold image to determine edges of cone (image derivative)

    private static final Scalar BLUE = new Scalar (0,0,255);
    private static final Scalar RED = new Scalar (255,0,0);
    boolean viewportPaused = false;

    public double blueThresh =180;
    public double HThreshLow = 100;
    public double HThreshHigh = 140;

    public double SThreshLow = 0;
    public double SThreshHigh = 254;

    public double VThreshLow = 0;
    public double VThreshHigh = 254;

    public int startColumn = 0;
    public int endColumn =239;
    public int startRow = 0;
    public int endRow =319;
    @Override
    public Mat processFrame(Mat input){
        Mat inputHSV = new Mat();
        Mat inputHSV_thresholded = new Mat();
        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);
        Core.inRange(inputHSV,new Scalar(HThreshLow,SThreshLow,VThreshLow),new Scalar(HThreshHigh,SThreshHigh,VThreshHigh),inputHSV_thresholded);

        int Xavg =0;

        Moments moments = Imgproc.moments(inputHSV_thresholded);


        for (int i=startRow; i<=endRow;i++){
            double rowAvg = 0;
            for (int j=startColumn; j<=endColumn;j++){
                rowAvg= rowAvg + inputHSV_thresholded.get(i,j)[0];
            }
            Xavg += rowAvg;
        }
        //Xavg= Xavg + ;
        Xavg = Xavg/(endColumn/startColumn);

        //inputHSV_thresholded.get();
        return inputHSV_thresholded;

    }
}
