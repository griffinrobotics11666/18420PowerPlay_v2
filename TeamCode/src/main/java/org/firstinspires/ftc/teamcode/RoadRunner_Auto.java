package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "drive")
public class RoadRunner_Auto extends LinearOpMode {

    OpenCvWebcam webcam;
    private final TherePipeline pipeline = new TherePipeline();
    TherePipeline.PowerPlayPosition getAnalysis = TherePipeline.PowerPlayPosition.LEFT;




    @Override
    public void runOpMode()  {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // First we need to establish our starting coordinates. We create a new Pose2d and assign it to our starting point.
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);

        // camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        // end camera stuff

// The next segment is a simple example. Notice how all the trajectories start where the last one ended or the beginning point(startPose, traj1.end, ect).
// If you cant figure out command to use visit:https://learnroadrunner.com/trajectorybuilder-functions.html#forward-distance-double


//    Trajectory traj1 = drive.trajectoryBuilder(startPose)
//            .lineToLinearHeading(new Pose2d(46, 0, Math.toRadians(-90)))
//            .build();
//
//    Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//            .lineToLinearHeading(new Pose2d(46, -23, Math.toRadians(0)))
//            .build();
//
//    Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//            .lineToLinearHeading(new Pose2d(23, -23, Math.toRadians(180)))
//            .build();
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectory(traj1);
//
//        drive.followTrajectory(traj2);
//
//        drive.followTrajectory(traj3);



// It works but we have a lot of code. Using TrajectorySequence we can simplify it.
// Notice on the Dashboard, it is only shown as a single path unlike the previous example.
// Make sure to use .waitSeconds() NOT .wait() if you want a delay.

    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(new Pose2d(46, 0, Math.toRadians(-90)))
            .lineToLinearHeading(new Pose2d(46, -23, Math.toRadians(0)))
// Like this .waitSeconds(500)
            .lineToLinearHeading(new Pose2d(23, -23, Math.toRadians(180)))
            .strafeRight(23)
            .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
            .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);

//Here is a simple example with the webcam.
//
//    Trajectory traj1 = drive.trajectoryBuilder(startPose)
//            .lineToLinearHeading(new Pose2d(46, 0, Math.toRadians(-90)))
//            .build();
//
//    Trajectory traj2 = drive.trajectoryBuilder(startPose)
//            .lineToLinearHeading(new Pose2d(23, 0, Math.toRadians(-90)))
//            .build();
//
//    Trajectory traj3 = drive.trajectoryBuilder(startPose)
//            .splineToConstantHeading(new Vector2d(23,0), Math.toRadians(0))
//            .splineToConstantHeading(new Vector2d(23,-23), Math.toRadians(0))
//            .build();
//
//
//        waitForStart();
//
//        if (!isStopRequested())
//            getAnalysis = pipeline.getAnalysis();
//            sleep(1000);
//            switch (getAnalysis) {
//                case LEFT: { //one
//                    if (!isStopRequested())
//                        drive.followTrajectory(traj1);
//                    break;
//
//                }
//
//                case CENTER: { //two
//                    if (!isStopRequested())
//                        drive.followTrajectory(traj2);
//                   break;
//                }
//
//
//                case RIGHT: { //three
//                    if (!isStopRequested())
//                        drive.followTrajectory(traj3);
//                    break;
//                }
//
//            }
//
//
    }


}

