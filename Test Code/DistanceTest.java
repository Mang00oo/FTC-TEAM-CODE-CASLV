package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Distance Test", group="Linear OpMode")

public class DistanceTest extends LinearOpMode {

    DistanceSensor distance;
    
    @Override
    public void runOpMode() {
        
        distance = hardwareMap.get(DistanceSensor.class, "DistanceSensor");

        waitForStart();
        
        while (opModeIsActive()) {

            telemetry.addData("Distance", Double.toString(distance.getDistance(DistanceUnit.CM)));
            telemetry.update();
            
        }
    }
}
