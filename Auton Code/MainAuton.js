//Import necessary packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="MainAuton", group="Main Code")

public class MainAuton extends LinearOpMode {
    private IMU imuAsIMU;
    private double robotYaw;
    private double leftPower;
    private double rightPower;
    private double targetRot;
    private double tick;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private double distance_M;
    private DistanceSensor distance;
    private ColorSensor color;
    private double hspeed;
    private double vspeed;
    private double codeStep;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();;

    @Override
    public void runOpMode() {
        
        // SET VARIABLES
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;
        robotYaw = 0;
        leftPower = 0;
        rightPower = 0;
        targetRot = 0;
    
        hspeed = 0;
        vspeed = 0;
    
        codeStep = 0;
        
        // Assign Motors
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            
        // Set Motor Directions
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            
        // SETUP SENSORS
        imuAsIMU = hardwareMap.get(IMU.class, "imu");
        distance = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        color = hardwareMap.get(ColorSensor.class, "colorSensor");
            
        //INITIALIZE IMU
        imuAsIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        orientation = imuAsIMU.getRobotYawPitchRollAngles();
        angularVelocity = imuAsIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
        robotYaw = orientation.getYaw(AngleUnit.DEGREES);
        
        telemetry.addData("IMU status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        imuAsIMU.resetYaw();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get the orientation and angular velocity.
            orientation = imuAsIMU.getRobotYawPitchRollAngles();
            angularVelocity = imuAsIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            robotYaw = orientation.getYaw(AngleUnit.DEGREES);
        
            distance_M = distance.getDistance(DistanceUnit.CM);
        
            // Sets correct power for left and right sides of motors
            if (robotYaw > targetRot){
                leftPower = ((targetRot + robotYaw)/80)+vspeed;
                rightPower = ((targetRot + robotYaw)/-80)+vspeed;
            }
            if (robotYaw < targetRot){
                rightPower = ((targetRot + robotYaw)/-80)+vspeed;
                leftPower = ((targetRot + robotYaw)/80)+vspeed;
            }
            
            // Update Tick
            tick = tick +1;
            
            // AUTON SEQUENCE
            
            // Step 1: Move Forward
            if (codeStep == 0)
            {
                vspeed = 0.3;
                if (tick > 40)
                {
                vspeed = 0;
                tick = 0;
                codeStep = 1;
                }
            }
            
            // Step 2: Check and Move
            if (codeStep == 1)
            {
                // Check Middle Pos.
                if (distance_M < 40)
                {
                    // Move forward
                    vspeed = 0.3;
                    if (tick > 20)
                    {
                        vspeed = 0;
                        tick = 0;
                        targetRot = 0;
                        codeStep = 2;
                    }
                }
                else
                {
                    // Check Right Pos.
                    //Turn
                    vspeed = 0;
                    targetRot = 60;
                    if(tick>30)
                    {
                        //Check
                        if (distance_M < 40)
                        {
                            //Move
                            vspeed = 0.3;
                            if (tick > 40)
                            {
                                // Stop Moving
                                vspeed = 0;
                                tick = 0;
                                codeStep = 2;
                            }
                        }
                        else
                        {
                            // Go Left
                            // Turn
                            vspeed = 0;
                            targetRot = -60;
                            if(tick>20)
                            {
                                // Move
                                vspeed = 0.3;
                                if (tick > 40)
                                {
                                    // Stop Moving
                                    vspeed = 0;
                                    tick = 0;
                                    codeStep = 2;
                                }
                            }
                        }
                    }
                }
            }
            
            
            
            
            
            // Set Motor Powers
            frontLeft.setPower(-leftPower);
            frontRight.setPower(-rightPower);
            backLeft.setPower(-leftPower);
            backRight.setPower(-rightPower);
            
            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Tick", tick);
            telemetry.addData("Code Step", codeStep);
            telemetry.update();
        }
    }
}
