package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/*class Powers {
    public final static int Time = 0;
    public final static int XPower = 1;
    public final static int YPower = 2;
    public final static int Rotation = 3;
    public final static int ArmPower = 4;
    public final static int HandPower = 5;
}*/

@Autonomous
public class Autonommoos04 extends LinearOpMode{
    // Variables to store motors
    private DcMotor drive0; 
    private DcMotor drive1; 
    private DcMotor drive2; 
    private DcMotor drive3; 
    
    // Variable to store servos
    private Servo servo0;
    private Servo servo1;
    private Servo servo2;

    // Variables for gamepad 
    boolean rightBumperPressed = false; 
    boolean leftBumperPressed = false; 
    
    double speed = 0.5; 
    double acceleration = 0.1;

    double y;// Remember, Y stick is reversed! (this is also copied we just moved it) 
    double x; 
    double rotation;  
    
    double servoDirection;
    
    /*double[][] speedChanges = {
        // Timings (in frames)
        {  0,  22,  60},//,  20,  20,  30,  20,  20,  55,  10, 150,   0,   0,   0,   0,   0,   0,   0,   0},
        // X position
        {  0,   0,   0,  -1,   0,   0,  -1,   1,   0,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0},
        // Y position
        {  0,   1,   0,   0,   1,  -1,   0,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0},
        // Rotation speeds
        {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
        // Arm power
        {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
        // Grabber power
        {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}
    };*/
    
    double[][] speedChanges = {
        {   0, 120,  30, 120},//6,  60,  15},
        {   0,   0,   0,   1,  0,   0,   0},
        {   0,   0, 0.1,   0,   0,   0,   1},
        {   0,   0,   0,   0,   0,   0,   0},
        {   0.5,   0.5,   0.5,   0.5, -1,   -1,   0,   0},
        {   1,   1,   1,   1,   1,   1,   1}
    };

    @Override
    public void runOpMode(){
        waitForStart();
        
        // Init the hardware and variables and stuff
        y = 0;
        x = 0; 
        rotation = 0; 

        servoDirection = 0;

        // Gets hardware info from the setup files
        drive0 = hardwareMap.get(DcMotor.class, "drive0");  //front l 
        drive1 = hardwareMap.get(DcMotor.class, "drive1");  //front r 
        drive2 = hardwareMap.get(DcMotor.class, "drive2");  //back l 
        drive3 = hardwareMap.get(DcMotor.class, "drive3");  //back r 

        servo0 = hardwareMap.get(Servo.class, "servo0");    // Arm height servo
        servo1 = hardwareMap.get(Servo.class, "servo1");    // Hand servo
        servo2 = hardwareMap.get(Servo.class, "servo2");    // Hand servo
        
        int frameCount = 0;
        int mode = 0;
        
        // Game loop
        while (opModeIsActive()){
            if (frameCount >= speedChanges[Powers.Time][mode]){
                frameCount = 0;
                mode++;
                if (speedChanges[Powers.Time].length <= mode){
                    return;
                }
                x = speedChanges[Powers.XPower][mode];
                y = speedChanges[Powers.YPower][mode];
                rotation = speedChanges[Powers.Rotation][mode];
            }
            
            updateMotorPowers();
            servo0.setPosition(speedChanges[Powers.ArmPower][mode]);
            servo1.setPosition(speedChanges[Powers.HandPower][mode]);
            
            telemetryUpdate(speed);
            
            // Simulate 30 "frames" per second
            wait(33);
            frameCount++;
        }
    }
    
    // Wait for the given number of milliseconds
    public void wait(int ms){
        try {
            Thread.sleep(ms);
        } catch (Exception e){
            Thread.currentThread().interrupt();
        }
    }
    
    // Update the powers of the motors
    public void updateMotorPowers(){ 
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
    
    // Send info back to the driver hub
    public void telemetryUpdate(double speed){ 
        telemetry.addData("speed", speed);
        telemetry.addData("acceleration", acceleration);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rotation", rotation);
        telemetry.addData("servoDirection", servoDirection);
        telemetry.addData("", ""); 
        telemetry.addData("drive0", drive0.getPower()); 
        telemetry.addData("drive1", drive1.getPower()); 
        telemetry.addData("drive2", drive2.getPower()); 
        telemetry.addData("drive3", drive3.getPower());
        telemetry.addData("servo0", servo0.getPosition());
        telemetry.addData("servo1", servo1.getPosition());
        telemetry.update(); 
    }
}