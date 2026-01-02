package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest01 extends LinearOpMode{
    DistanceSensor testDistance;
    @Override
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()) {
            testDistance = hardwareMap.get(DistanceSensor.class, "distance0");
            double distance = testDistance.getDistance(DistanceUnit.CM);
            telemetry.addData("distance", distance);
            telemetry.update();
        }
    }
}