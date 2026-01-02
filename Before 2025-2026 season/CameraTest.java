package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.*;
import com.qualcomm.robotcore.hardware.*;

@TeleOp

public class CameraTest extends LinearOpMode{
    AprilTagProcessor aprilTags;
    VisionPortal visionPortal;
    List<AprilTagDetection> aprilTagDetections;
    
    @Override
    public void runOpMode () {
        aprilTags = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "webcam0"),
            aprilTags);
        while (opModeIsActive()) {
            getDetections();
            updateTelemetry();
        }
    }
    
    public void getDetections() {
        aprilTagDetections = aprilTags.getDetections();
    }
    
    public void updateTelemetry() {
        for (AprilTagDetection detection : aprilTagDetections) {
            telemetry.addData("AprilTag", "" + detection.id);
            telemetry.addData("x", detection.rawPose.x);
            telemetry.addData("y", detection.rawPose.y);
            telemetry.addData("z", detection.rawPose.z);
            telemetry.addData("pitch", detection.ftcPose.pitch);
            telemetry.addData("roll", detection.ftcPose.roll);
            telemetry.addData("yaw", detection.ftcPose.yaw);
        }
    }
}