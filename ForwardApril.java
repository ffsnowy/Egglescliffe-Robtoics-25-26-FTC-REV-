package org.firstinspires.ftc.teamcode;//if you code this in bluejay this line will disappear btw

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
public class ForwardApril extends LinearOpMode{
    //variables to store motors and servos when we get the servos  
    private DcMotor drive0;
    private DcMotor drive1;
    private DcMotor drive2;
    private DcMotor drive3;
    private DcMotor drive4;
    private DcMotor drive5;
    
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
   
    // IMU for field-centric control
    private IMU imu;
    private double robotHeading = 0;
    private double headingOffset = 0;

    //variables for gamepad
    boolean rightBumperPressed = false;
    boolean leftBumperPressed = false;
    //boolean gamepad1BPressed = false;
    boolean rightTriggerPressed = false;
    boolean leftTriggerPressed = false;
    boolean resetHeadingPressed = false;
   
    double speed = 0;
    double acceleration = 0.1;

    double y;// Remember, Y stick is reversed! (this is also copied we just moved it)
    double x;
    double rotation;  
   
    double armDirection;
   
    double ascentDir = 0.0f;

    @Override
    public void runOpMode(){

        boolean streamVision = true;
        
        waitForStart();
        
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam0"), myAprilTagProcessor);
       
        if (!streamVision){
            myVisionPortal.stopStreaming();
        }
       
        y = -gamepad1.left_stick_y;// Remember, Y stick is reversed! (this is also copied we just moved it)
        x = gamepad1.left_stick_x;
        rotation = -gamepad1.right_stick_x;

        armDirection = gamepad2.right_stick_y;

        //gets info from the setup files which allows sensors motors servos ect to be ran

        distance0 = hardwareMap.get(DistanceSensor.class, "distance0");

        drive0 = hardwareMap.get(DcMotor.class, "drive0");//front l
        drive1 = hardwareMap.get(DcMotor.class, "drive1");//front r
        drive2 = hardwareMap.get(DcMotor.class, "drive2");//back l
        drive3 = hardwareMap.get(DcMotor.class, "drive3");//back r
       
        drive4 = hardwareMap.get(DcMotor.class, "drive4"); // ascent
        drive5 = hardwareMap.get(DcMotor.class, "drive5"); // arm

        servo0 = hardwareMap.get(Servo.class, "servo0"); // Arm height servo
        servo1 = hardwareMap.get(Servo.class, "servo1"); // Hand servo
        //servo2 = hardwareMap.get(Servo.class, "servo2"); // Hand servo

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
       
        // Define how the hub is oriented on the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
       
        imu.initialize(parameters);
       
        // Reset heading offset when starting
        resetHeading();

        //drive0.setDirection(DcMotorSimple.Direction.REVERSE);
        //drive1.setDirection(DcMotorSimple.Direction.REVERSE);
        //drive2.setDirection(DcMotorSimple.Direction.REVERSE);
        //drive3.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()){ //loops when the game is running

            //distance = distance0.getDistance(DistanceUnit.CM);

            speed = speedUpdate(speed);
            updateHeading();

            //multiplier + also allows us to slow down the movement of the robot

            getInput();//updates those inputs
            movement();
            setServoSpeed();
            updateGrabber();
            telemetryAprilTag();
            telemetryUpdate(speed);
            wait(10);

            //updates telemetry with new info so we can debug easier

        }
         
        //this works from the old code and works really well so i'm just gonna copy it icl and change the variable names
        //right nvm i'm not copying the whole thing so much for "only relates to speed" smh
        //if the subroutine changes purpose MAKE A NEW ONE OR RENAME THE SUBROUTINE FOR GODS SAKE

    }
   
    // Reset the heading to 0 (call this when robot is facing away from drivers)
    public void resetHeading() {
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
   
    // Update the current robot heading
    public void updateHeading() {
        robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
    }

    // Set servo rotation speed based on joystick
    public void setServoSpeed(){
        drive5.setPower(armDirection);
        //servo2.setPosition(servoDirection / 2 + 0.5);
        //drive4.setPower(ascentDir);
        servo0.setPosition(ascentDir/2 + 0.5);
    }

    //stops speed updating once a tick and makes it once a button press through the use of a boolean

    public double speedUpdate(double speed){

        final double speedAddition = 0.1 * speed;

        //that's what it was in the og code and it works well so we won't change it for now
        //see how it's a final bc it gets used often in this subroutine and i only have to change it once to change it all the times it's used? very mindful, very demure, very cutesy

        if (gamepad1.right_bumper){//object created not by us we can't rename that
            if (! rightBumperPressed){
                speed = speed + speedAddition;
            }
            rightBumperPressed = true;
        } else {
            rightBumperPressed = false;
        }

        if (gamepad1.left_bumper){//object created not by us we can't rename that
            if (! leftBumperPressed){
                speed = speed - speedAddition;
            }
            leftBumperPressed = true;
        } else {
            leftBumperPressed = false;
        }
       
        if (gamepad1.left_trigger > 0.1){
            if (!leftTriggerPressed){
                speed = 0.1;
            }
            leftTriggerPressed = true;
        } else {
            leftTriggerPressed = false;
        }
       
        if (gamepad1.right_trigger > 0.1){
            if (!rightTriggerPressed){
                speed = 0.7;
            }
            rightTriggerPressed = true;
        } else {
            rightTriggerPressed = false;
        }
       
        // Reset heading with Y button 
        if (gamepad1.y) {
            if (!resetHeadingPressed) {
                resetHeading();
            }
            resetHeadingPressed = true;
        } else {
            resetHeadingPressed = false;
        }

        if (speed > 1){
            speed = 1;
        } else if (speed < 0.2){
            speed = 0.2;
        }
        return speed;
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
        //ascentDir = 0;
        //if (gamepad2.left_bumper){
         //   ascentDir--;
        //}
        //if (gamepad2.right_bumper){
          //  ascentDir++;
       
        //}
        ascentDir = gamepad2.left_stick_y;
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

    public void movement(){
        //Field-centric movement: rotate joystick inputs by robot heading
       
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
       
        double frontLeftPower = -drive0.getPower() + goalFrontLeftPower;
        double frontRightPower = -drive1.getPower() + goalFrontRightPower;
        double backLeftPower = -drive2.getPower() + goalBackLeftPower;
        double backRightPower = -drive3.getPower() + goalBackRightPower;
       
        double maxPower = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
       
        // If maxPower is below the threshold, set to goal value and just let it be.
        if (maxPower <= 0.1){
            drive0.setPower(goalFrontLeftPower);
            drive1.setPower(goalFrontRightPower);
            drive2.setPower(goalBackLeftPower);
            drive3.setPower(goalBackRightPower);
            return;
        }
       
        double frontLeftChange = frontLeftPower / maxPower * acceleration;
        double frontRightChange = frontRightPower / maxPower * acceleration;
        double backLeftChange = backLeftPower / maxPower * acceleration;
        double backRightChange = backRightPower / maxPower * acceleration;
       
        drive0.setPower(drive0.getPower() + frontLeftChange);
        drive1.setPower(drive1.getPower() + frontRightChange);
        drive2.setPower(drive2.getPower() + backLeftChange);
        drive3.setPower(drive3.getPower() + backRightChange);
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

    public void telemetryUpdate(double speed) // Adds all of the telemetry info we want and then updates the telemetry info displayed
    {
        telemetry.addData("distance (cm)", distance);
        telemetry.addData("speed", speed);
        telemetry.addData("acceleration", acceleration);
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotHeading));
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rotation", rotation);
        telemetry.addData("armDirection", armDirection);
        telemetry.addData("", "");
        telemetry.addData("drive0", drive0.getPower());
        telemetry.addData("drive1", drive1.getPower());
        telemetry.addData("drive2", drive2.getPower());
        telemetry.addData("drive3", drive3.getPower());
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
