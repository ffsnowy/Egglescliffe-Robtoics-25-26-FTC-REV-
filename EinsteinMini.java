package org.firstinspires.ftc.teamcode.einstein;

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
public class EinsteinMini extends LinearOpMode{
  private DriveSystem drive_system;
  double x, y, rotation = 0;
  
  boolean leftBumperPressed, rightBumperPressed = false;
  boolean resetHeadingPressed = false;
  
  @Override
  public void runOpMode(){
    waitForStart();
    drive_system = new DriveSystem(
      hardwareMap.get(DcMotor.class, "drive0"),
      hardwareMap.get(DcMotor.class, "drive1"),
      hardwareMap.get(DcMotor.class, "drive2"),
      hardwareMap.get(DcMotor.class, "drive3"),
      hardwareMap.get(IMU.class, "imu")
    );
    
    while (opModeIsActive()){
      getInput();
      updateSpeed();
      drive_system.updateHeading();
      drive_system.setMotorPowers(x, y, rotation);
      drive_system.telemetry(telemetry);
      updateTelemetry();
      
      wait(10);
    }
  }
  
  public void wait(int ms){
    try {
      Thread.sleep(ms);
    } catch (Exception e){
      Thread.currentThread().interrupt();
    }
  }
  
  public void getInput(){
    getVirtualJoysticks();
  }
  
  public void getVirtualJoysticks(){
    y = -gamepad1.left_stick_y;// Remember, Y stick is reversed! (this is also copied we just moved it)
    x = gamepad1.left_stick_x;
    rotation = -gamepad1.right_stick_x;
  }
  
  void updateSpeed() {
    if (gamepad1.right_bumper){//object created not by us we can't rename that
      if (!rightBumperPressed){
        drive_system.setSpeed(1.0);
      }
      rightBumperPressed = true;
    } else {
      rightBumperPressed = false;
    }

    if (gamepad1.left_bumper){//object created not by us we can't rename that
      if (! leftBumperPressed){
        drive_system.setSpeed(0.2);
      }
      leftBumperPressed = true;
    } else {
      leftBumperPressed = false;
    }
    
    if (gamepad1.y) {
      if (!resetHeadingPressed) {
        drive_system.resetHeading();
      }
      resetHeadingPressed = true;
    } else {
      resetHeadingPressed = false;
    }
  }
  public void updateTelemetry() {
    telemetry.addData("x", x);
    telemetry.addData("y", y);
    telemetry.addData("rotation", rotation);
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
  
  
  
  
  
  
  
