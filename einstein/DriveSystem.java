package org.firstinspires.ftc.teamcode.einstein;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class DriveSystem {
  private DcMotor frontLeftDrive;
  private DcMotor frontRightDrive;
  private DcMotor backLeftDrive;
  private DcMotor backRightDrive;

  // IMU for field-centric control
  private IMU imu;
  private double robotHeading = 0;
  private double headingOffset = 0;

  /* Multipliers that affect movement */
  // Speed will keep itself between 0.2 and 1.0
  private double speed = 0.2;
  // How much the motor powers can change per call of setMotorPowers
  private double acceleration = 0.1;

  public DriveSystem(
      DcMotor frontLeftDrive,
      DcMotor frontRightDrive,
      DcMotor backLeftDrive,
      DcMotor backRightDrive,
      IMU imu
  ) {
    this.frontLeftDrive = frontLeftDrive;
    this.frontRightDrive = frontRightDrive;
    this.backLeftDrive = backLeftDrive;
    this.backRightDrive = backRightDrive;

    // Initialize IMU
    this.imu = imu;
     
    // Define how the hub is oriented on the robot
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
     
    imu.initialize(parameters);
     
    // Reset heading offset when starting
    resetHeading();
  }

  public double incrementSpeed() {
    speed += 0.1;
    if (speed < 0.2) {
      speed = 0.2;
    }
    if (speed > 1) {
      speed = 1;
    }
    return speed;
  }
  public double decrementSpeed() {
    speed -= 0.1;
    if (speed < 0.2) {
      speed = 0.2;
    }
    if (speed > 1) {
      speed = 1;
    }
    return speed;
  }
  public double getSpeed() {
    return speed;
  }
  public double setSpeed(double newSpeed) {
    speed = newSpeed;
    if (speed < 0.2) {
      speed = 0.2;
    }
    if (speed > 1) {
      speed = 1;
    }
    return speed;
  }

  public void resetHeading() {
    headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
  }
  public void updateHeading() {
    robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
  }

  public void setMotorPowers(double x, double y, double rotation) {
    final double strafeCorrection = 1.1; //this is up to driver preferences so can be modified as needed, tho if different people like different numbers make a list or smth bc we ain't changing it mid comp, make different copies of the same code for different people or smth

    //I might have an idea to find an objectively correct value for straife correction, see me (Lawrence) for details
    
    // Apply field-centric transformation
    double cosHeading = Math.cos(robotHeading);
    double sinHeading = -Math.sin(robotHeading);
     
    // Rotate the joystick inputs by the robot's heading
    double fieldCentricX = x * cosHeading - y * sinHeading;
    double fieldCentricY = x * sinHeading + y * cosHeading;
     
    // Apply strafe correction to the field-centric X
    double adjustedx = strafeCorrection * fieldCentricX;

    double denominator = (Math.max(Math.abs(fieldCentricY) + Math.abs(adjustedx) + Math.abs(rotation), 1)) / speed;

    // Goal motor powers using field-centric coordinates
    double goalFrontLeftPower = (fieldCentricY + adjustedx + rotation) / denominator;
    double goalBackLeftPower = (fieldCentricY - adjustedx + rotation) / denominator;
    double goalFrontRightPower = (fieldCentricY - adjustedx - rotation) / denominator;
    double goalBackRightPower = (fieldCentricY + adjustedx - rotation) / denominator;

    //the pluses and minuses may be subject to change i think depending how the motors are set up
    //the pluses and minuses have been changed and so should? Work now (testing needed)
     
    double frontLeftPower = -frontLeftDrive.getPower() + goalFrontLeftPower;
    double frontRightPower = -frontRightDrive.getPower() + goalFrontRightPower;
    double backLeftPower = -backLeftDrive.getPower() + goalBackLeftPower;
    double backRightPower = -backRightDrive.getPower() + goalBackRightPower;
     
    double maxPower = Math.max(
      Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
      Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
    );
     
    // If maxPower is below the threshold, set to goal value and just let it be.
    if (maxPower <= 0.1){
      frontLeftDrive.setPower(goalFrontLeftPower);
      frontRightDrive.setPower(goalFrontRightPower);
      backLeftDrive.setPower(goalBackLeftPower);
      backRightDrive.setPower(goalBackRightPower);
      return;
    }
     
    double frontLeftChange = frontLeftPower / maxPower * acceleration;
    double frontRightChange = frontRightPower / maxPower * acceleration;
    double backLeftChange = backLeftPower / maxPower * acceleration;
    double backRightChange = backRightPower / maxPower * acceleration;
     
    frontLeftDrive.setPower(frontLeftDrive.getPower() + frontLeftChange);
    frontRightDrive.setPower(frontRightDrive.getPower() + frontRightChange);
    backLeftDrive.setPower(backLeftDrive.getPower() + backLeftChange);
    backRightDrive.setPower(backRightDrive.getPower() + backRightChange);
  }
  public void telemetry(Telemetry telemetry) {
    telemetry.addLine("DriveSystem Telemetry START");
    telemetry.addData("speed", speed);
    telemetry.addData("acceleration", acceleration);
    telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotHeading));
    telemetry.addData("frontLeftDrive", frontLeftDrive.getPower());
    telemetry.addData("frontRightDrive", frontRightDrive.getPower());
    telemetry.addData("backLeftDrive", backLeftDrive.getPower());
    telemetry.addData("backRightDrive", backRightDrive.getPower());
    telemetry.addLine("DriveSystem Telemetry END");
  }
}