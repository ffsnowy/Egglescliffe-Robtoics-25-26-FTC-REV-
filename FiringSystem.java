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

public class FiringSystem {
  private DcMotor launcher;
  private boolean firing;
  public FiringSystem(DcMotor launcher) {
    this.launcher = launcher;
    launcher.setTargetPosition(-2500);
    launcher.setPower(0);
    launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
  public void update() {
    if (!firing) {
      launcher.setTargetPosition(-2500);
      launcher.setPower(0);
      launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      return;
    }
    launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    if (launcher.getCurrentPosition() < launcher.getTargetPosition()) {
      launcher.setPower(0);
      launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      firing = false;
    }
    return;
  }
  public void fire() {
    launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    launcher.setPower(-1);
    firing = true;
  }
  
  public void telemetry(Telemetry telemetry) {
    telemetry.addLine("FiringSystem Telemetry START");
    telemetry.addData("launcher", launcher.getCurrentPosition());
    telemetry.addLine("FiringSystem Telemetry END");
  }
    
}




