//Main Driver code for FTC Team _Teamnumber_

//Import packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MainDriver")
public class MainDriver extends LinearOpMode {

  private DcMotor backRight;
  private DcMotor backLeft;
  private DcMotor frontLeft;
  private DcMotor frontRight;
  
  private DcMotor arm;
  
  private Servo shooterServo;
  private Servo armRot;
  private Servo claw;
  //private Servo ClawRight;
  //private Servo ClawLeft;
  
  private IMU imuAsIMU;
  
  private YawPitchRollAngles orientation;
  private double yaw;
  
  private double aSin;
  private double aCos;
  private double angle;
  
  private boolean kaboom;
  private boolean clawOpen;
  
  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    //Get hardware
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    
    arm = hardwareMap.get(DcMotor.class, "arm");
    
    shooterServo = hardwareMap.get(Servo.class, "shooterServo");
    armRot = hardwareMap.get(Servo.class, "claw");
    claw = hardwareMap.get(Servo.class, "clawRot");
    //ClawLeft = hardwareMap.get(Servo.class, "ClawLeft");
    //ClawRight = hardwareMap.get(Servo.class, "ClawRight");
    
    //Set motor directions
    //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
    //Gyro Stuff
    imuAsIMU = hardwareMap.get(IMU.class, "imu");
    AngularVelocity angularVelocity;
    imuAsIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    //Reset Yaw
    imuAsIMU.resetYaw();
    
    //Main Loop
    waitForStart();
    if (opModeIsActive()) {
      
      //Set Variables
      double flp = 0;
      double frp = 0;
      double blp = 0;
      double brp = 0;
      
      double speed = 0.7;
      double turn = 0.3;
      
      double clawdir = 0;
      
      while (opModeIsActive()) {
        
        //Get Orientation
        orientation = imuAsIMU.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw(AngleUnit.DEGREES);
        aSin = Math.sin(yaw);
        aCos = Math.cos(yaw);
        angle = Math.atan2((0.001*gamepad1.left_stick_y - 0.001*gamepad1.left_stick_x), (0.001*gamepad1.left_stick_x + 0.001*gamepad1.left_stick_y));

        arm.setPower((gamepad1.right_trigger * 0.9)-(gamepad1.left_trigger * 50));
        
        if(gamepad1.right_trigger > 0)
        {
          armRot.setPosition(0);
        }
        if(gamepad1.left_trigger > 0)
        {
          armRot.setPosition(1);
        }
        
        //Rotational Movement
        flp = flp-gamepad1.right_stick_x * turn;
        frp = frp+gamepad1.right_stick_x * turn;
        blp = blp-gamepad1.right_stick_x * turn;
        brp = brp+gamepad1.right_stick_x * turn;
        
        //Vertical Movement
        flp = flp+gamepad1.left_stick_y * speed;
        frp = frp+gamepad1.left_stick_y * speed;
        blp = blp+gamepad1.left_stick_y * speed;
        brp = brp+gamepad1.left_stick_y * speed;
        
        //Horizontal Movement
        flp = (flp - gamepad1.left_stick_x) * speed;
        frp = (frp + gamepad1.left_stick_x) * speed;
        blp = (blp + gamepad1.left_stick_x) * speed;
        brp = (brp - gamepad1.left_stick_x) * speed;
        
        //Set motor powers
        frontLeft.setPower(flp);
        frontRight.setPower(frp);
        backLeft.setPower(blp);
        backRight.setPower(brp);
        
        if (gamepad1.circle == true)
        {
          kaboom = true;
        }
        
        if(kaboom)
        {
          shooterServo.setPosition(0);
        }
        else
        {
          shooterServo.setPosition(1);
        }
        
        if (gamepad1.cross == true)
        {
          clawOpen = true;
        }
        if (gamepad1.triangle == true)
        {
          clawOpen = false;
        }
        
        //clawdir = clawdir + (gamepad1.right_stick_y / 10);
        
        //Move Claw
        //ClawLeft.setPosition(clawdir);
        //ClawRight.setPosition(clawdir);

        telemetry.addData("Yaw", yaw);
        telemetry.addData("ANGLE", angle);
        telemetry.update();
      }
    }
  }
}
