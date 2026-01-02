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
public class Autonommoos07 extends LinearOpMode{
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
    
    double speed = 1.0; 
    double acceleration = 0.1;

    double y = 0.0f;// Remember, Y stick is reversed! (this is also copied we just moved it) 
    double x = 0.0f; 
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
        int stage = 0;
        
        //int armAim = 1300;
        int armAim = 1300;
        
        float[][] instructions = {
            // {x, y, rotation, arm, grabber, time}
            {0.0f, 0.5f, 0.0f, 0.0f, 1.0f, 5, 0},
            {0.0f, 0.0f, 0.0f, armAim, 1.0f, 150, 0},
            {0.0f, 0.4f, 0.0f, armAim, 1.0f, 5, 0},
            {0.0f, 0.5f, 0.0f, armAim -115, 1.0f, 20, 0},
            {0.0f, -0.4f, 0.0f, armAim, 0.0f, 7, 0},
            {0.0f, -0.5f, 0.0f, armAim, 0.0f, 30, 0},
            {0.6f, 0.0f, 0.0f, armAim, 0.0f, 60, 0},
            {0.0f, 0.0f, 0.0f, armAim, 0.0f, -1, 0}
        };
    
        while (opModeIsActive()){ //loops when the game is running 
            int currentPos = drive5.getCurrentPosition();
            x = instructions[stage][0];
            y = instructions[stage][1];
            rotation = instructions[stage][2];
            movement();
            servo1.setPosition(instructions[stage][4]);
            if (currentPos < startArmDirection + instructions[stage][3] -5){
                if (instructions[stage][6] == 0){
                    drive5.setPower(0.5f);
                } else {
                    drive5.setPower(0.4f);
                }
            } else if (currentPos > startArmDirection + instructions[stage][3] +5){
                if (instructions[stage][6] == 0){
                    drive5.setPower(-0.5f);
                } else {
                    drive5.setPower(-0.4f);
                }
            } else {
                drive5.setPower(0.0f);
            }
            frameCounter++;
            if (frameCounter > instructions[stage][5] && instructions[stage][5] != -1){
                frameCounter = 0;
                stage++;
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
    
     public void movement(){ 
        //ok this was just copied and variables changed, so don't ask me how it works, i can't visualise bro 
        //meant to cover moving + strafing + turns i think 

        final double strafeCorrection = 1.1; //this is up to driver preferences so can be modified as needed, tho if different people like different numbers make a list or smth bc we ain't changing it mid comp, make different copies of the same code for different people or smth 

        //I might have an idea to find an objectively correct value for straife correction, see me (Lawrence) for details 
        double adjustedx = strafeCorrection * x;

        double denominator = (Math.max(Math.abs(y) + Math.abs(adjustedx) + Math.abs(rotation), 1)) / speed; 

        // Goal motor powers
        double goalFrontLeftPower = (y + adjustedx + rotation) / denominator; 
        double goalBackLeftPower = (y - adjustedx + rotation) / denominator; 
        double goalFrontRightPower = (y - adjustedx - rotation) / denominator; 
        double goalBackRightPower = (y + adjustedx - rotation) / denominator; 

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
        if (1 == 1){//maxPower <= 0.1){
            drive0.setPower(goalFrontLeftPower);
            drive1.setPower(goalFrontRightPower);
            drive2.setPower(goalBackLeftPower);
            drive3.setPower(goalBackRightPower);
            telemetry.addData("Working?", "yes");
            telemetry.update();
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