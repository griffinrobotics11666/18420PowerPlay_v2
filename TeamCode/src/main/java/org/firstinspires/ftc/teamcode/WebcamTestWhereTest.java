
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Webcam Test Where (New)", group = "Concept")

public class WebcamTestWhereTest extends OpMode {
  OpenCvWebcam webcam;
  WherePipeline pipeline = new WherePipeline();
  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
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

    telemetry.addLine("Waiting for start");
    telemetry.update();
  }

  @Override
  public void init_loop() {
    telemetry.addData("Frame Count", webcam.getFrameCount());
    telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
    telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
    telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
    telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
    telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
    telemetry.addData("Hue Min Threshold", pipeline.HThreshLow);
    telemetry.addData("Hue Max Threshold", pipeline.HThreshHigh);
    telemetry.addData("Saturation Min Threshold", pipeline.SThreshLow);
    telemetry.addData("Saturation Max Threshold", pipeline.SThreshHigh);
    telemetry.addData("Value Min Threshold", pipeline.VThreshLow);
    telemetry.addData("Value Max Threshold", pipeline.VThreshHigh);
    telemetry.addData("Xavg Value", pipeline.getAnalysis());
    telemetry.update();

    if (gamepad1.dpad_up && pipeline.HThreshLow < 255) {
      pipeline.HThreshLow += .05;
    }
    if (gamepad1.dpad_down && pipeline.HThreshLow > 0) {
      pipeline.HThreshLow -= .05;
    }

    if (gamepad2.dpad_up && pipeline.HThreshHigh < 255) {
      pipeline.HThreshHigh += .05;
    }
    if (gamepad2.dpad_down && pipeline.HThreshHigh > 0) {
      pipeline.HThreshHigh -= .05;
    }

    if (gamepad1.a && pipeline.SThreshLow < 255) {
      pipeline.SThreshLow += .05;
    }
    if (gamepad1.b && pipeline.SThreshLow > 0) {
      pipeline.SThreshLow -= .05;
    }

    if (gamepad2.a&& pipeline.SThreshHigh < 255) {
      pipeline.SThreshHigh += .05;
    }
    if (gamepad2.b && pipeline.SThreshHigh > 0) {
      pipeline.SThreshHigh -= .05;
    }

    if (gamepad1.x && pipeline.VThreshLow < 255) {
      pipeline.VThreshLow += .05;
    }
    if (gamepad1.y && pipeline.VThreshLow > 0) {
      pipeline.VThreshLow -= .05;
    }

    if (gamepad2.x && pipeline.VThreshHigh < 255) {
      pipeline.VThreshHigh += .05;
    }
    if (gamepad2.y && pipeline.VThreshHigh > 0) {
      pipeline.VThreshHigh -= .05;
    }


    if (gamepad2.dpad_up && pipeline.endRow < 319) {
    pipeline.startRow += .05;
    }
    if (gamepad2.y && pipeline.endRow > 0) {
    pipeline.endRow -= .05;
    }
  }

  @Override
  public void start() {
    runtime.reset();
  }

  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
  }
}
