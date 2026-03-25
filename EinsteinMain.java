package org.firstinspires.ftc.teamcode.einstein;//if you code this in bluejay this line will disappear btw

import android.util.Size;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import java.lang.Math;  
import java.util.List;

@TeleOp
public class EinsteinMain extends LinearOpMode{

  // Drive System
  private DriveSystem driveSystem;

  // Other Motors
  private DcMotor drive4;
  private DcMotor drive5;
  private DcMotor drive6;
  private DcMotor drive7;
  
  // Camera vision thing
  AprilTagProcessor myAprilTagProcessor;
  private VisionPortal myVisionPortal;
   
  // Variable to store servo
  private Servo servo0;
  private Servo servo1;
  //private Servo servo2;
   
  // Distance sensor
  private DistanceSensor distance0;
  private double distance;
   
  //variables for gamepad
  boolean rightBumperPressed = false;
  boolean leftBumperPressed = false;
  //boolean gamepad1BPressed = false;
  boolean rightTriggerPressed = false;
  boolean leftTriggerPressed = false;
  boolean resetHeadingPressed = false;
  // Timing variables for trigger controls
  long infeederStartTime = 0;
  boolean infeederRunning = false;
  
  // Outfeeder state machine
  int outfeederState = 0; // 0=off, 1=spinup, 2=feeding, 3=cooldown
  long outfeederStateStartTime = 0;
  
  double y;// Remember, Y stick is reversed! (this is also copied we just moved it)
  double x;
  double rotation;  
   
  double armDirection;
   
  double ascentDir = 0.0f;



  @Override
  public void runOpMode(){

    boolean streamVision = true;
    
    waitForStart();
   
    // DriveSystem for managing the mechanum wheels
    driveSystem = new DriveSystem(
      hardwareMap.get(DcMotor.class, "drive0"),
      hardwareMap.get(DcMotor.class, "drive1"),
      hardwareMap.get(DcMotor.class, "drive2"),
      hardwareMap.get(DcMotor.class, "drive3"),
      hardwareMap.get(IMU.class, "imu")
    );
    
    /*myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam0"), myAprilTagProcessor);
     
    if (!streamVision){
      myVisionPortal.stopStreaming();
    }*/
     
    y = -gamepad1.left_stick_y;// Remember, Y stick is reversed! (this is also copied we just moved it)
    x = gamepad1.left_stick_x;
    rotation = -gamepad1.right_stick_x;

    armDirection = gamepad2.right_stick_y;

    //gets info from the setup files which allows sensors motors servos ect to be ran

    distance0 = hardwareMap.get(DistanceSensor.class, "distance0");

    drive4 = hardwareMap.get(DcMotor.class, "drive4"); // ascent
    drive5 = hardwareMap.get(DcMotor.class, "drive5"); // arm
    drive6 = hardwareMap.get(DcMotor.class, "drive6"); // ascent
    drive7 = hardwareMap.get(DcMotor.class, "drive7"); // arm

    servo0 = hardwareMap.get(Servo.class, "servo0"); // Arm height servo
    servo1 = hardwareMap.get(Servo.class, "servo1"); // Hand servo

    while (opModeIsActive()){ //loops when the game is running

      // Update distance reading
      //distance = distance0.getDistance(DistanceUnit.CM);

      // Get gamepad input
      getInput();
      // Update speed multiplier
      updateSpeed();
      // Update drive heading
      driveSystem.updateHeading();
      // Update motor powers
      driveSystem.setMotorPowers(x, y, rotation);
      // Control launcher
      ligma();
      // Control Infeeder
      infeed();
      // Update servo powers
      setServoSpeed();
      // Update grabber servo power
      updateGrabber();

      // Send debug telemetry to the driver hub
      //telemetryAprilTag();
      driveSystem.telemetry(telemetry);
      updateTelemetry();

      wait(10);
    }
     
    //this works from the old code and works really well so i'm just gonna copy it icl and change the variable names
    //right nvm i'm not copying the whole thing so much for "only relates to speed" smh
    //if the subroutine changes purpose MAKE A NEW ONE OR RENAME THE SUBROUTINE FOR GODS SAKE

  }

  // Set servo rotation speed based on joystick
  public void setServoSpeed(){
    drive5.setPower(armDirection);
    //servo2.setPosition(servoDirection / 2 + 0.5);
    //drive4.setPower(ascentDir);
    servo0.setPosition(ascentDir/2 + 0.5);
  }

  //stops speed updating once a tick and makes it once a button press through the use of a boolean

  public void updateSpeed(){

    if (gamepad1.right_bumper){//object created not by us we can't rename that
      if (!rightBumperPressed){
        driveSystem.setSpeed(1.0);
      }
      rightBumperPressed = true;
    } else {
      rightBumperPressed = false;
    }

    if (gamepad1.left_bumper){//object created not by us we can't rename that
      if (! leftBumperPressed){
        driveSystem.setSpeed(0.2);
      }
      leftBumperPressed = true;
    } else {
      leftBumperPressed = false;
    }
  
  /*   
    if (gamepad1.left_trigger > 0.1){
      if (!leftTriggerPressed){
        ######################;
      }
      leftTriggerPressed = true;
    } else {
      leftTriggerPressed = false;
    }
     
    if (gamepad1.right_trigger > 0.1){
      if (!rightTriggerPressed){
        #######################;
      }
      rightTriggerPressed = true;
    } else {
      rightTriggerPressed = false;
    }
*/
     
    // Reset heading with Y button 
    if (gamepad1.y) {
      if (!resetHeadingPressed) {
        driveSystem.resetHeading();
      }
      resetHeadingPressed = true;
    } else {
      resetHeadingPressed = false;
    }
  }

  //the hard stuff now we've included mecanum stuff so we gotta actually figure this out

  public void getVirtualJoysticks(){
    //law wanted this for auto
    y = -gamepad1.left_stick_y;// Remember, Y stick is reversed! (this is also copied we just moved it)
    x = gamepad1.left_stick_x;
    rotation = -gamepad1.right_stick_x;
  }
   
  // Update input variables
  // block code superior

  public void getInput(){
    getVirtualJoysticks();
    armDirection = gamepad2.right_stick_y;
    ascentDir = gamepad2.left_stick_y;
    
    // Left trigger - simple infeed for set duration
    if (leftTriggerPressed){
      leftTriggerPressed = gamepad1.left_trigger > 0.1;
    } else {
      if (gamepad1.left_trigger > 0.1){
        // Start infeeder
        infeederRunning = true;
        infeederStartTime = System.currentTimeMillis();
        leftTriggerPressed = true;
      }
    }
    
    // Right trigger - outfeeder sequence
    if (rightTriggerPressed){
      rightTriggerPressed = gamepad1.right_trigger > 0.1;
      // If trigger released and we're in active states, move to cooldown
      if (!rightTriggerPressed && (outfeederState == 1 || outfeederState == 2)){
        outfeederState = 3;
        outfeederStateStartTime = System.currentTimeMillis();
      }
    } else {
      if (gamepad1.right_trigger > 0.1){
        // Start outfeeder sequence
        outfeederState = 1;
        outfeederStateStartTime = System.currentTimeMillis();
        rightTriggerPressed = true;
      }
    }
  }
  
  public void ligma(){
    long currentTime = System.currentTimeMillis();
    long elapsed = currentTime - outfeederStateStartTime;
    
    double ligmaspeed = 0.43; //CALIBRATION - outfeeder motor speed
    
    switch(outfeederState){
      case 0: // Off
        drive6.setPower(0);
        drive7.setPower(0);
        break;
        
      case 1: // Spinup phase - outfeeder only
        drive6.setPower(-ligmaspeed);
        drive7.setPower(ligmaspeed);
        if (elapsed > 2000){ //CALIBRATION - spinup time in milliseconds (2 seconds)
          outfeederState = 2;
          outfeederStateStartTime = currentTime;
        }
        break;
        
      case 2: // Feeding phase - outfeeder continues, infeeder pushes ball
        drive6.setPower(-ligmaspeed);
        drive7.setPower(ligmaspeed);
        // Infeeder is handled in infeed() method
        if (elapsed > 1000){ //CALIBRATION - feeding duration (1 second)
          // If trigger still pressed, stay in this state
          // If trigger released, cooldown is triggered in getInput()
        }
        break;
        
      case 3: // Cooldown - outfeeder spins down
        drive6.setPower(-ligmaspeed);
        drive7.setPower(ligmaspeed);
        if (elapsed > 500){ //CALIBRATION - cooldown time (0.5 seconds)
          outfeederState = 0;
        }
        break;
    }
  }
  
  public void infeed(){
    long currentTime = System.currentTimeMillis();
    
    // Handle left trigger infeed
    if (infeederRunning){
      long elapsed = currentTime - infeederStartTime;
      if (elapsed < 4000){ //CALIBRATION - infeed duration (4 seconds)
        drive5.setPower(1);
      } else {
        drive5.setPower(0);
        infeederRunning = false;
      }
      return;
    }
    
    // Handle right trigger outfeeder sequence infeed
    if (outfeederState == 2){
      long elapsed = currentTime - outfeederStateStartTime;
      if (elapsed < 1000){ //CALIBRATION - how long infeeder pushes ball (1 second)
        drive5.setPower(1);
      } else {
        drive5.setPower(0);
      }
      return;
    }
    
    // Default - motor off
    drive5.setPower(0);
  }
   
  public void updateGrabber(){
    final double correction = 0.001;
    if (gamepad2.b == true){
      servo1.setPosition(1.0);
    }
    if (gamepad2.a == true){
      servo1.setPosition(0.1);
    }
  }

  public void wait(int ms){
    try {
      Thread.sleep(ms);
    } catch (Exception e){
      Thread.currentThread().interrupt();
    }
  }

  private void telemetryAprilTag() {
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
  
    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      telemetry.addLine("");
      if (myAprilTagDetection.metadata != null) {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
      }
    }
    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
  }

  public void updateTelemetry() {
    telemetry.addData("distance (cm)", distance);
    telemetry.addData("x", x);
    telemetry.addData("y", y);
    telemetry.addData("rotation", rotation);
    telemetry.addData("armDirection", armDirection);
    telemetry.addData("", "");
    telemetry.addData("drive4", drive4.getPower());
    telemetry.addData("drive5", drive5.getPower());
    telemetry.addData("drive5 Position", drive5.getCurrentPosition());
    //telemetry.addData("servo0", servo0.getPosition());
    telemetry.addData("servo1", servo1.getPosition());
    telemetry.addData("", "");
    telemetry.addData("buttonA1", gamepad1.a);
    telemetry.addData("buttonB1", gamepad1.b);
    telemetry.addData("buttonX1", gamepad1.x);
    telemetry.addData("buttonY1 (Reset Heading)", gamepad1.y);
    telemetry.addData("bumperR1", gamepad1.right_bumper);
    telemetry.addData("bumperL1", gamepad1.left_bumper);
    telemetry.addData("triggerR1", gamepad1.right_trigger);
    telemetry.addData("triggerL1", gamepad1.left_trigger);
    telemetry.addData("joystickXR1", gamepad1.right_stick_x);
    telemetry.addData("joystickYR1", gamepad1.right_stick_y);
    telemetry.addData("joystickXL1", gamepad1.left_stick_x);
    telemetry.addData("joystickYL1", gamepad1.left_stick_y);
    telemetry.addData("", "");
    telemetry.addData("buttonA2", gamepad2.a);
    telemetry.addData("buttonB2", gamepad2.b);
    telemetry.addData("buttonX2", gamepad2.x);
    telemetry.addData("buttonY2", gamepad2.y);
    telemetry.addData("bumperR2", gamepad2.right_bumper);
    telemetry.addData("bumperL2", gamepad2.left_bumper);
    telemetry.addData("triggerR2", gamepad2.right_trigger);
    telemetry.addData("triggerL2", gamepad2.left_trigger);
    telemetry.addData("joystickXR2", gamepad2.right_stick_x);
    telemetry.addData("joystickYR2", gamepad2.right_stick_y);
    telemetry.addData("joystickXL2", "BROKEN (" + gamepad2.left_stick_x + ")");
    telemetry.addData("joystickYL2", "BROKEN (" + gamepad2.left_stick_y + ")");
    telemetry.update();
  }
}
