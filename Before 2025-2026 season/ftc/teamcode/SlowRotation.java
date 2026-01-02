package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class SlowRotation extends LinearOpMode{

    // 
    DcMotor drive4;
    
    @Override
    public void runOpMode(){
        
        waitForStart();
        drive4 = hardwareMap.get(DcMotor.class, "drive4");
        
        while (opModeIsActive()){
            drive4.setPower(1);
            
            telemetry.addData("drive4", drive4.getPower());
            telemetry.update();
        }
    }
}

            
            
            
