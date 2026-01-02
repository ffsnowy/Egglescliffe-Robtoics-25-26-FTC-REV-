package org.firstinspires.ftc.teamcode;//if you code this in bluejay this line will disappear btw 
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; 
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor; 
import com.qualcomm.robotcore.hardware.DcMotorSimple; 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; 
import com.qualcomm.robotcore.hardware.Servo; 
import java.lang.Math;

@Autonomous
public class Autonommoos06 extends LinearOpMode{
    //variables to store motors and servos when we get the servos 
    // nathan is jizz-tastic
    private DcMotor drive0; 
    private DcMotor drive1; 
    private DcMotor drive2; 
    private DcMotor drive3;
    private DcMotor drive4;
    private DcMotor drive5;
    
    // Variable to store servo
    //private Servo servo0;
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
    
    double speed = 0; 
    double acceleration = 0.1;

    double y;// Remember, Y stick is reversed! (this is also copied we just moved it) 
    double x; 
    double rotation;  
    
    double armDirection;
    double startArmDirection;
    
    double ascentDir = 0.0f;
    
    public void runOpMode(){ 

        waitForStart();
        
        y = -gamepad1.left_stick_y;// Remember, Y stick is reversed! (this is also copied we just moved it) 
        x = gamepad1.left_stick_x; 
        rotation = -gamepad1.right_stick_x; 

        armDirection = gamepad2.right_stick_y;

        //gets info from the setup files which allows sensors motors servos ect to be ran 

        distance0 = hardwareMap.get(DistanceSensor.class, "distance0");

        servo1 = hardwareMap.get(Servo.class, "servo1");
        drive0 = hardwareMap.get(DcMotor.class, "drive0");//front l 
        drive1 = hardwareMap.get(DcMotor.class, "drive1");//front r 
        drive2 = hardwareMap.get(DcMotor.class, "drive2");//back l 
        drive3 = hardwareMap.get(DcMotor.class, "drive3");//back r
        
        drive4 = hardwareMap.get(DcMotor.class, "drive4"); // ascent 
        drive5 = hardwareMap.get(DcMotor.class, "drive5"); // arm
        
        startArmDirection = drive5.getCurrentPosition();

        //servo0 = hardwareMap.get(Servo.class, "servo0"); // Arm height servo
        //servo1 = hardwareMap.get(Servo.class, "servo1"); // Hand servo
    
        int frameCounter = 0;
    
        while (opModeIsActive()){ //loops when the game is running 
            int currentPos = drive5.getCurrentPosition();
            int armAim = 1300;
            if (frameCounter < 7){
                drive0.setPower(0.7f);
                drive1.setPower(0.7f);
                drive2.setPower(0.7f);
                drive3.setPower(0.7f);
                frameCounter++;
            } else if (frameCounter < 20) {
                drive0.setPower(0.0f);
                drive1.setPower(0.0f);
                drive2.setPower(0.0f);
                drive3.setPower(0.0f);
                if (currentPos < startArmDirection + armAim -5){
                    drive5.setPower(0.5f);
                } else if (currentPos > startArmDirection + armAim +5){
                    drive5.setPower(-0.5f);
                } else {
                    drive5.setPower(0.0f);
                    frameCounter++;
                }
            } else if (frameCounter < 25){
                drive0.setPower(0.4f);
                drive1.setPower(0.4f);
                drive2.setPower(0.4f);
                drive3.setPower(0.4f);
                if (currentPos < startArmDirection + armAim -5){
                    drive5.setPower(0.5f);
                } else if (currentPos > startArmDirection + armAim +5){
                    drive5.setPower(-0.5f);
                } else {
                    drive5.setPower(0.0f);
                }
                frameCounter++;
            } else if (frameCounter < 40){
                drive0.setPower(0.4f);
                drive1.setPower(0.4f);
                drive2.setPower(0.4f);
                drive3.setPower(0.4f);
                if (currentPos < startArmDirection + armAim - 75 -5){
                    drive5.setPower(1.0f);
                } else if (currentPos > startArmDirection + armAim - 75 +5){
                    drive5.setPower(-1.0f);
                } else {
                    drive5.setPower(0.0f);
                }
                frameCounter++;
            } else if (frameCounter < 45){
                drive0.setPower(-0.4f);
                drive1.setPower(-0.4f);
                drive2.setPower(-0.4f);
                drive3.setPower(-0.4);
                servo1.setPosition(1.0f);
                if (currentPos < startArmDirection + armAim -5){
                    drive5.setPower(1.0f);
                } else if (currentPos > startArmDirection + armAim +5){
                    drive5.setPower(-1.0f);
                } else {
                    drive5.setPower(0.0f);
                }
                frameCounter++;
            } else {
                drive0.setPower(0.0f);
                drive1.setPower(0.0f);
                drive3.setPower(0.0f);
                drive2.setPower(0.0f);
                servo1.setPosition(0.1f);
                if (currentPos < startArmDirection + armAim -5){
                    drive5.setPower(0.5f);
                } else if (currentPos > startArmDirection + armAim +5){
                    drive5.setPower(-0.5f);
                } else {
                    drive5.setPower(0.0f);
                }
                frameCounter++;
            }
            telemetryUpdate(0.0f);
            wait(30);
        }
    }
    
    public void wait(int ms){
        try {
            Thread.sleep(ms);
        } catch (Exception e){
            Thread.currentThread().interrupt();
        }
    }
    
    public void telemetryUpdate(double speed) // Adds all of the telemetry info we want and then updates the telemetry info displayed
    { 
        telemetry.addData("distance (cm)", distance);
        telemetry.addData("speed", speed);
        telemetry.addData("acceleration", acceleration);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rotation", rotation);
        telemetry.addData("armDirection", armDirection);
        telemetry.addData("startArmDirection", startArmDirection);
        telemetry.addData("", ""); 
        telemetry.addData("drive0", drive0.getPower());
        telemetry.addData("drive0 pos", drive0.getCurrentPosition()); 
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
        telemetry.addData("buttonY1", gamepad1.y); 
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