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
  private Servo shooter;
  //private Servo ClawLeft;
  
  private DcMotor arm;
  private Servo clawTop;
  private Servo clawBottom;
  private Servo clawPivot;
  private Servo armChute;
  
  
  private IMU imuAsIMU;
  
  private YawPitchRollAngles orientation;
  private double yaw;
  
  private double aSin;
  private double aCos;
  private double angle;
  
  private boolean kaboom;
  
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
	shooter = hardwareMap.get(Servo.class, "shooter");
	
	arm = hardwareMap.get(DcMotor.class, "armMotor");
	
	clawTop = hardwareMap.get(Servo.class, "clawTop");
	clawBottom = hardwareMap.get(Servo.class, "clawBottom");
	clawPivot = hardwareMap.get(Servo.class, "clawPivot");
	armChute = hardwareMap.get(Servo.class, "armChute");
	
	
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
	  
	  double speed = 0.5;
	  double turn = 0.4;
	  
	  kaboom = false;
	  
	  double clawdir = 0;
	  
	  while (opModeIsActive()) {
		
		//Get Orientation
		orientation = imuAsIMU.getRobotYawPitchRollAngles();
		yaw = orientation.getYaw(AngleUnit.DEGREES);
		aSin = Math.sin(yaw);
		aCos = Math.cos(yaw);
		angle = Math.atan2((0.001*gamepad1.left_stick_y - 0.001*gamepad1.left_stick_x), (0.001*gamepad1.left_stick_x + 0.001*gamepad1.left_stick_y));

		arm.setPower((gamepad1.left_trigger * 0.4)-(gamepad1.right_trigger * 0.4));
		
		if (gamepad1.dpad_up)
		{
			clawBottom.setPosition(1);
			clawPivot.setPosition(0.8);
		}
		if (gamepad1.dpad_down)
		{
			clawBottom.setPosition(0.44);
			clawPivot.setPosition(0.55);
		}
		
		if (gamepad1.dpad_left)
		{
			clawTop.setPosition(0.24);
		}
		if (gamepad1.dpad_right)
		{
			clawTop.setPosition(0);
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
		
		if(gamepad1.circle)
		{
		  kaboom = true;
		}
		if(kaboom)
		{
		  shooter.setPosition(0.9);
		} else {
		  shooter.setPosition(0.5);
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
