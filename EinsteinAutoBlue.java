package org.firstinspires.ftc.teamcode.einstein;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.List;

@Autonomous(name="Einstein Auto Blue")
public class EinsteinAutoBlue extends LinearOpMode{

  // Drive System
  private DriveSystem driveSystem;

  // Other Motors
  private DcMotor drive4;
  private DcMotor drive5;
  private DcMotor drive6;
  private DcMotor drive7;
  
  // Camera vision thing
  private AprilTagProcessor myAprilTagProcessor;
  private VisionPortal myVisionPortal;
  
  // Variable to store servo
  private Servo servo0;
  private Servo servo1;
   
  // Distance sensor
  private DistanceSensor distance0;
  private double distance;
  
  // Seeing the tag the robot should go to 
  AprilTagDetection myGoalTag = null;
  AprilTagDetection obeliskTag = null;
  
  //variables for gamepad
  boolean rightBumperPressed = false;
  boolean leftBumperPressed = false;
  boolean rightTriggerPressed = false;
  boolean leftTriggerPressed = false;
  boolean resetHeadingPressed = false;

  double y;
  double x;
  double rotation;  
   
  double armDirection;
  double ascentDir = 0.0f;
  
  boolean moving = false;
  String firingStatus = "Invalid";
  
  private final double SPEED_MULTIPLIER = 1.0;
  
  // CALIBRATION: Blue team AprilTag ID (check manual)
  private final int BLUE_APRILTAG_ID = 20;
  
  // CALIBRATION: Obelisk AprilTag IDs
  private final int OBELISK_TAG_1 = 21;
  private final int OBELISK_TAG_2 = 22;
  private final int OBELISK_TAG_3 = 23;
  
  // CALIBRATION: Stage 1 - Approach obelisk parameters
  private final double TARGET_OBELISK_DISTANCE = 50.0; // inches from obelisk
  private final double OBELISK_DISTANCE_TOLERANCE = 10.0; // inches
  private final double APPROACH_OBELISK_SPEED = 0.25; // Speed when approaching obelisk
  
  // CALIBRATION: Firing zone parameters (in inches and degrees)
  private final double MIN_FIRING_DISTANCE = 35.0; // inches
  private final double MAX_FIRING_DISTANCE = 45.0; // inches
  private final double TARGET_FIRING_DISTANCE = 40.0; // inches
  private final double MAX_YAW_ERROR = 10.0; // degrees
  private final double MAX_X_OFFSET = 10.0; // inches (side to side)
  
  // CALIBRATION: Movement parameters
  private final double APPROACH_SPEED = 0.7; // Speed when approaching target
  private final double SEARCH_ROTATION_SPEED = 0.2; // Speed when searching for target
  private final double POSITION_TOLERANCE = 20.0; // inches
  private final double YAW_TOLERANCE = 10.0; // degrees
  
  // CALIBRATION: Firing parameters
  private final double OUTFEEDER_SPEED = 0.43;
  private final int FIRING_TIME_MS = 800; // Time per ball in milliseconds
  private final int BALLS_TO_FIRE = 3;

  @Override
  public void runOpMode(){

    boolean streamVision = true;
    
    // DriveSystem for managing the mechanum wheels
    driveSystem = new DriveSystem(
      hardwareMap.get(DcMotor.class, "drive0"),
      hardwareMap.get(DcMotor.class, "drive1"),
      hardwareMap.get(DcMotor.class, "drive2"),
      hardwareMap.get(DcMotor.class, "drive3"),
      hardwareMap.get(IMU.class, "imu")
    );

    myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam0"), myAprilTagProcessor);
     
    if (!streamVision){
      myVisionPortal.stopStreaming();
    }
    
    while (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
      wait(50);
    }
     
    y = 0;
    x = 0;
    rotation = 0;

    armDirection = 0;

    distance0 = hardwareMap.get(DistanceSensor.class, "distance0");

    // drive4 = hardwareMap.get(DcMotor.class, "drive4");
    // drive5 = hardwareMap.get(DcMotor.class, "drive5");
    // drive6 = hardwareMap.get(DcMotor.class, "drive6");
    // drive7 = hardwareMap.get(DcMotor.class, "drive7");

    servo0 = hardwareMap.get(Servo.class, "servo0");
    servo1 = hardwareMap.get(Servo.class, "servo1");

    telemetry.addData("Status", "Initialized - Blue Team");
    telemetry.addData("Target AprilTag", BLUE_APRILTAG_ID);
    telemetry.update();
    
    waitForStart();

    ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
    exposureControl.setExposure(6, java.util.concurrent.TimeUnit.MILLISECONDS);
    GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
    gainControl.setGain(100);

    int stage = 1;
    int firingCounter = 0;
    int ballsFired = 0;

    while (opModeIsActive()){

      // Find tags
      findObeliskTag();
      findBlueAprilTag();
      
      // Check if we're in valid firing position
      checkFiringValidity();

      switch (stage) {
        case 1: {
          // Stage 1: Approach the obelisk to get closer to field center
          telemetry.addData("Stage", "1: Approaching Obelisk");
          
          if (obeliskTag != null) {
            // Move towards obelisk
            approachObelisk();
            
            // Check if we're at target distance from obelisk
            if (obeliskTag.ftcPose != null && 
                Math.abs(TARGET_OBELISK_DISTANCE - obeliskTag.ftcPose.y) < OBELISK_DISTANCE_TOLERANCE &&
                Math.abs(obeliskTag.ftcPose.x) < MAX_X_OFFSET &&
                Math.abs(obeliskTag.ftcPose.yaw) < YAW_TOLERANCE
            ) {
              // We're at the right distance, move to next stage
              stage++;
              x = 0;
              y = 0;
              rotation = 0;
            }
          } else {
            // No obelisk tag found, just move forward
            x = 0;
            y = APPROACH_OBELISK_SPEED; // CALIBRATION: forward speed if no tag
            rotation = 0;
          }
          break;
        }
        
        case 2: {
          // Stage 2: Rotate to search for blue target tag
          telemetry.addData("Stage", "2: Searching for Blue Target");
          telemetry.addData("Goal Tag Found", myGoalTag != null);
          
          if (myGoalTag != null && myGoalTag.ftcPose != null) {
            if (Math.abs(myGoalTag.ftcPose.bearing) < 3) {
              // Found the blue target, move to next stage
              telemetry.addData("Stage 2 Status", "TARGET FOUND! Moving to Stage 3");
              stage++;
              x = 0;
              y = 0;
              rotation = 0;
            } else {
              // Found the blue target, but has a big bearing
              telemetry.addData("Stage 2 Status", "TARGET FOUND! Adjusting bearing");
              x = 0;
              y = 0;
              rotation = SEARCH_ROTATION_SPEED;
            }
          } else {
            // Rotate to search for target
            telemetry.addData("Stage 2 Status", "Searching...");
            x = 0;
            y = 0;
            rotation = SEARCH_ROTATION_SPEED; // CALIBRATION: search rotation speed
          }
          break;
        }
        
        case 3: {
          // Stage 3: Approach the blue target tag
          telemetry.addData("Stage", "3: Approaching Blue Target");
          telemetry.addData("Goal Tag Present", myGoalTag != null);
         
          double distance_cm = distance0.getDistance(DistanceUnit.CM);
          //hello noah
          double ideal_distance = 50;
          double tolerance = 3.141592653589793;
          double speeeeeeeeeeeeeeeEEEEEEEEEEEEED = 0.3141592;
          if (distance_cm > ideal_distance + tolerance) {
            //"This means we're too far away, so move forwards", Noah 24.02.2026
            x = 0;
            y = speeeeeeeeeeeeeeeEEEEEEEEEEEEED;
            rotation = 0;
          }
          
          if (distance_cm < ideal_distance - tolerance) {
            //"This means we're too close, so move backwards", Noah 24.02.2026
            x = 0;
            y = -speeeeeeeeeeeeeeeEEEEEEEEEEEEED;
            rotation = 0;
          }
          
          if (Math.max(
              ideal_distance - tolerance,
              Math.min(ideal_distance + tolerance, distance_cm)) == distance_cm
          ) {
            //"If the distance is in the valid range, proceed to the next stage", Noah 24.02.2026
            x = 0;
            y = 0;
            rotation = 0;
            stage++;
          }
          
          break;
        }
        
        case 4: {
          // Stage 4: Fire balls
          telemetry.addData("Stage", "4: Firing");
          
          x = 0;
          y = 0;
          rotation = 0;
          
          if (stage == 4) {
            break;
          }
          
          // Run outfeeder
          drive6.setPower(-OUTFEEDER_SPEED);
          drive7.setPower(OUTFEEDER_SPEED);
          
          firingCounter++;
          
          // Fire for calculated time
          if (firingCounter >= (FIRING_TIME_MS / 10)) { // Divided by 10 because loop is 10ms
            ballsFired++;
            firingCounter = 0;
            
            if (ballsFired >= BALLS_TO_FIRE) {
              stage++;
            }
          }
          break;
        }
        
        case 5: {
          // Stage 5: Stop and finish
          telemetry.addData("Stage", "5: Complete");
          x = 0;
          y = 0;
          rotation = 0;
          drive6.setPower(0);
          drive7.setPower(0);
          break;
        }
      }
      
      
      driveSystem.setSpeed(SPEED_MULTIPLIER);
      
      // Update drive heading
      ///driveSystem.updateHeading();
      // Update motor powers
      driveSystem.setMotorPowers(x, y, rotation);

      // Send debug telemetry to the driver hub
      telemetry.addData("Firing Status", firingStatus);
      telemetry.addData("Balls Fired", ballsFired + "/" + BALLS_TO_FIRE);
      telemetryAprilTag();
      driveSystem.telemetry(telemetry);
      updateTelemetry();

      wait(10);
    }
  }

  // Find any obelisk AprilTag
  private void findObeliskTag() {
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
  
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    
    obeliskTag = null;
    
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      
      // Accept any obelisk tag
      if (myAprilTagDetection.id == OBELISK_TAG_1 || 
          myAprilTagDetection.id == OBELISK_TAG_2 || 
          myAprilTagDetection.id == OBELISK_TAG_3) {
        obeliskTag = myAprilTagDetection;
        break;
      }
    }
  }

  // Find the blue AprilTag and ignore red tags
  private void findBlueAprilTag() {
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
  
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    
    myGoalTag = null;
    
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      
      // Only accept the blue AprilTag
      if (myAprilTagDetection.id == BLUE_APRILTAG_ID) {
        myGoalTag = myAprilTagDetection;
        break; // Found our tag, stop looking
      }
    }
  }

  // Approach the obelisk to get to field center
  private void approachObelisk() {
    if (obeliskTag == null || obeliskTag.ftcPose == null) {
      x = 0;
      y = APPROACH_OBELISK_SPEED; // CALIBRATION: default forward if no pose
      rotation = 0;
      return;
    }
    
    // CALIBRATION: Proportional control for obelisk approach
    double kP_y_obelisk = 0.02;
    double kP_x_obelisk = 0.05;
    
    // Calculate errors
    double errorY = TARGET_OBELISK_DISTANCE - obeliskTag.ftcPose.y;
    double errorX = -obeliskTag.ftcPose.x;
    
    // Calculate speeds
    y = -errorY * kP_y_obelisk;
    x = -errorX * kP_x_obelisk;
    
    rotation = obeliskTag.ftcPose.yaw / 45;
    rotation = Math.min(1, Math.max(rotation, -1));
    
    if (obeliskTag.ftcPose.bearing > YAW_TOLERANCE ||
        obeliskTag.ftcPose.bearing < -YAW_TOLERANCE) {
      rotation = 0;          
    }
    
    // Limit speeds
    double maxSpeed = APPROACH_OBELISK_SPEED;
    x = Math.max(-maxSpeed, Math.min(maxSpeed, x));
    y = Math.max(-maxSpeed, Math.min(maxSpeed, y));
  }

  // Check if robot is in valid firing position
  private void checkFiringValidity() {
    if (myGoalTag == null) {
      firingStatus = "Invalid - No Tag";
      return;
    }
    
    if (myGoalTag.ftcPose == null) {
      firingStatus = "Invalid - No Pose";
      return;
    }
    
    double distance = myGoalTag.ftcPose.y;
    double xOffset = Math.abs(myGoalTag.ftcPose.x);
    double yaw = Math.abs(myGoalTag.ftcPose.yaw);
    
    // Check all firing conditions
    if (distance < MIN_FIRING_DISTANCE) {
      firingStatus = "Invalid - Too Close";
      return;
    }
    
    if (distance > MAX_FIRING_DISTANCE) {
      firingStatus = "Invalid - Too Far";
      return;
    }
    
    if (Math.abs(xOffset) > MAX_X_OFFSET) {
      firingStatus = "Invalid - Off Center";
      return;
    }
    
    if (Math.abs(yaw) > MAX_YAW_ERROR) {
      firingStatus = "Invalid - Bad Angle";
      return;
    }
    firingStatus = "Valid";
  }

  // Calculate movement to approach the AprilTag
  private void calculateApproachMovement() {
    if (myGoalTag == null || myGoalTag.ftcPose == null) {
      moving = false;
      x = 0;
      y = 0;
      rotation = 0;
      return;
    }
    
    moving = true;
    
    // CALIBRATION: Proportional control gains
    double kP_x = 0.15; // Side-to-side correction strength
    double kP_y = 0.15; // Forward/back correction strength
    double kP_rotation = 0.05; // Rotation correction strength
    
    // Calculate errors
    double errorX = -myGoalTag.ftcPose.x;
    double errorY = TARGET_FIRING_DISTANCE - myGoalTag.ftcPose.y;
    double errorYaw = -myGoalTag.ftcPose.yaw;
    
    // Calculate proportional speeds
    x = errorX * kP_x;
    y = errorY * kP_y;
    rotation = errorYaw * kP_rotation;
    
    // If within tolerance, stop moving
    if (Math.abs(errorX) < POSITION_TOLERANCE) {
      x = 0;
    }
    if (Math.abs(errorY) < POSITION_TOLERANCE) {
      y = 0;
    }
    if (Math.abs(errorYaw) < YAW_TOLERANCE) {
      rotation = 0;
    }
    
    // Limit maximum speed
    double maxSpeed = APPROACH_SPEED;
    x = Math.max(-maxSpeed, Math.min(maxSpeed, x));
    y = Math.max(-maxSpeed, Math.min(maxSpeed, y));
    rotation = Math.max(-maxSpeed, Math.min(maxSpeed, rotation));
  }

  public void wait(int ms) {
    try {
      Thread.sleep(ms);
    } catch (Exception e) {
      Thread.currentThread().interrupt();
    }
  }

  private void telemetryAprilTag() {
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
  
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      telemetry.addLine("");
      
      if (myAprilTagDetection.metadata != null) {
        if (myAprilTagDetection.id == BLUE_APRILTAG_ID) {
          telemetry.addLine(">>> BLUE TARGET <<<");
        }
        if (myAprilTagDetection.id == OBELISK_TAG_1 || 
            myAprilTagDetection.id == OBELISK_TAG_2 || 
            myAprilTagDetection.id == OBELISK_TAG_3) {
          telemetry.addLine(">>> OBELISK <<<");
        }
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
      }
    }
  }

  public void updateTelemetry() {
    telemetry.addData("moving", moving);
    telemetry.addData("x", x);
    telemetry.addData("y", y);
    telemetry.addData("rotation", rotation);
    telemetry.addData("distance0", distance0.getDistance(DistanceUnit.CM));
    if (obeliskTag != null && obeliskTag.ftcPose != null) {
      telemetry.addData("Obelisk Distance", obeliskTag.ftcPose.y);
    }
    if (myGoalTag != null) {
      telemetry.addData("Blue Tag ID", myGoalTag.id);
      if (myGoalTag.ftcPose != null) {
        telemetry.addData("Blue Tag Distance", myGoalTag.ftcPose.y);
        telemetry.addData("Blue Tag X Offset", myGoalTag.ftcPose.x);
        telemetry.addData("Blue Tag Yaw", myGoalTag.ftcPose.yaw);
      } else {
        telemetry.addData("Blue Tag Pose", "NULL");
      }
    } else {
      telemetry.addData("Blue Tag", "NOT FOUND");
    }
    telemetry.update();
  }
}
