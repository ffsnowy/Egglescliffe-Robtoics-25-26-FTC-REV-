package org.firstinspires.ftc.teamcode;//if you code this in bluejay this line will disappear btw 

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp; 

import com.qualcomm.robotcore.hardware.DcMotor; 

import com.qualcomm.robotcore.hardware.DcMotorSimple; 

import com.qualcomm.robotcore.hardware.Servo; 

import java.lang.Math;  

 

@Autonomous 

 

public class Autonommoos01 extends LinearOpMode{ 

    //variables to store motors and servos when we get the servos  

    private DcMotor drive0; 

    private DcMotor drive1; 

    private DcMotor drive2; 

    private DcMotor drive3; 

    double speed = 0; 

    double y = 0;

    double x = 0; 

    double rotation = 0; 

     

    @Override 

 

    public void runOpMode(){ 

         

        //gets info from the setup files which allows sensors motors servos ect to be ran 

        drive0 = hardwareMap.get(DcMotor.class, "drive0");//front l 

        drive1 = hardwareMap.get(DcMotor.class, "drive1");//front r 

        drive2 = hardwareMap.get(DcMotor.class, "drive2");//back l 

        drive3 = hardwareMap.get(DcMotor.class, "drive3");//back r 

         

        drive1.setDirection(DcMotorSimple.Direction.REVERSE); //flips direction

        drive3.setDirection(DcMotorSimple.Direction.REVERSE); 

        waitForStart();
         

        while (opModeIsActive()){ //loops when the game is running 

            y = 0.1;
            movement();
            wait(2000);
            y = -0.1;
            movement();
            wait(2000);
            y = 0;
            x = 0.1;
            movement();
            wait(2000);
            x = -0.1;
            movement();
            wait(2000);
            x = 0;
            rotation = 0.1;
            movement();
            wait(2000);
            rotation = -0.1;
            movement();
            wait(2000);
            rotation = 0;
            telemetryUpdate();

            //updates telemetry with new info so we can debug easier 

        } 

         

        //this works from the old code and works really well so i'm just gonna copy it icl and change the variable names 

        //right nvm i'm not copying the whole thing so much for "only relates to speed" smh 

        //if the subroutine changes purpose MAKE A NEW ONE OR RENAME THE SUBROUTINE FOR GODS SAKE 

    } 
    
     

    //the hard stuff now we've included mecanum stuff so we gotta actually figure this out 

     

    public void movement(){ 

        //ok this was just copied and variables changed, so don't ask me how it works, i can't visualise bro 

        //meant to cover moving + strafing + turns i think 

        final double strafeCorrection = 1.1; //this is up to driver preferences so can be modified as needed, tho if different people like different numbers make a list or smth bc we ain't changing it mid comp, make different copies of the same code for different people or smth 

        //I might have an idea to find an objectively correct value for straife correction, see me (Lawrence) for details 

 

        double adjustedx = strafeCorrection * x;

        double denominator = Math.max(Math.abs(y) + Math.abs(adjustedx) + Math.abs(rotation), 1); 

        double frontLeftPower = (y - adjustedx + rotation) / denominator; 

        double frontRightPower = (y + adjustedx - rotation) / denominator; 

        double backLeftPower = (y + adjustedx + rotation) / denominator; 

        double backRightPower = (y - adjustedx - rotation) / denominator; 

        //the pluses and minuses may be subject to change i think depending how the motors are set up 

        //the pluses and minuses have been changed and so should? Work now (testing needed) 

         

        drive0.setPower(frontLeftPower); 

        drive1.setPower(frontRightPower); 

        drive2.setPower(backLeftPower); 

        drive3.setPower(backRightPower); 

    } 

     

    public void telemetryUpdate() // Adds all of the telemetry info we want and then updates the telemetry info displayed 

    { 

        telemetry.addData("speed", speed); 

        telemetry.addData("", ""); 

        telemetry.addData("drive0", drive0.getPower()); 

        telemetry.addData("drive1", drive1.getPower()); 

        telemetry.addData("drive2", drive2.getPower()); 

        telemetry.addData("drive3", drive3.getPower()); 

        telemetry.addData("", ""); 

    } 
    
     
    
    public void wait(int ms) 

    { 

        //not our code 

        try 

        { 

            Thread.sleep(ms); 

        }  

        catch (Exception ex) 

        { 

            Thread.currentThread().interrupt(); 

        } 

    } 

} 