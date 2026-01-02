package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "cameraTest5")
public class cameraTest5 extends LinearOpMode {

  /**
   * Describe this function...
   */
  @Override
  public void runOpMode() {
    VisionPortal.Builder myVisionPortalBuilder;
    boolean USE_WEBCAM;
    int RESOLUTION_WIDTH;
    int RESOLUTION_HEIGHT;
    VisionPortal myVisionPortal;
    boolean lastX;
    int frameCount;
    long capReqTime;
    boolean x;

    // EDIT THESE PARAMETERS AS NEEDED
    USE_WEBCAM = true;
    RESOLUTION_WIDTH = 640;
    RESOLUTION_HEIGHT = 480;
    // Create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam0"));
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    myVisionPortalBuilder.setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT));
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
    lastX = false;
    frameCount = 0;
    capReqTime = 0;
    while (!isStopRequested()) {
      x = gamepad1.x;
      if (x && !lastX) {
        // Save the next frame to the given file.
        myVisionPortal.saveNextFrameRaw("CameraFrameCapture-" + frameCount);
        frameCount += 1;
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        capReqTime = System.currentTimeMillis();
      }
      lastX = x;
      telemetry.addLine("######## Camera Capture Utility ########");
      telemetry.addLine(" > Resolution: " + RESOLUTION_WIDTH + "x" + RESOLUTION_HEIGHT);
      telemetry.addLine(" > Press X (or Square) to capture a frame");
      telemetry.addData("> Camera Status", myVisionPortal.getCameraState());
      if (capReqTime != 0) {
        telemetry.addLine("");
        telemetry.addLine("Captured Frame!");
      }
      if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000) {
        capReqTime = 0;
      }
      telemetry.update();
    }
  }
}
