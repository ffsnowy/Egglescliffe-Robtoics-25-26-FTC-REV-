package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Ligma extends LinearOpMode {
    private DcMotor drive6;
    private DcMotor drive7;
    
    @Override
    public void runOpMode(){
        drive6 = hardwareMap.get(DcMotor.class, "drive6");
        drive7 = hardwareMap.get(DcMotor.class, "drive7");
        
        waitForStart();
        
        while (opModeIsActive()){
            drive6.setPower(-.75);
            drive7.setPower(.75);
        }
    }
}