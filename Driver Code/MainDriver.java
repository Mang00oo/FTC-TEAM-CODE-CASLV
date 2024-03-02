//Main Driver code for FTC Team 19810

// Import packages
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
  
  private DcMotor armLeft;
  private DcMotor armRight;
  
  private DcMotor intakeTop;
  private DcMotor intakeBottom;
  
  private Servo clawPivotLeft;
  private Servo clawPivotRight;
  private Servo claw;
  
  private Servo shooter;
  
  private Servo suspensionLeft;
  private Servo suspensionRight;
  
  @Override
  public void runOpMode() {
	
	// Get hardware
	backRight = hardwareMap.get(DcMotor.class, "backRight");
	backLeft = hardwareMap.get(DcMotor.class, "backLeft");
	frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
	frontRight = hardwareMap.get(DcMotor.class, "frontRight");
	
	shooter = hardwareMap.get(Servo.class, "shooter");
	
	armLeft = hardwareMap.get(DcMotor.class, "armLeft");
	armRight = hardwareMap.get(DcMotor.class, "armRight");
	
	intakeTop = hardwareMap.get(DcMotor.class, "intakeTop");
	intakeBottom = hardwareMap.get(DcMotor.class, "intakeBottom");
	
	claw = hardwareMap.get(Servo.class, "claw");
	clawPivotLeft = hardwareMap.get(Servo.class, "clawPivotLeft");
	clawPivotRight = hardwareMap.get(Servo.class, "clawPivotRight");
	
	suspensionRight = hardwareMap.get(Servo.class, "suspensionRight");
	suspensionLeft = hardwareMap.get(Servo.class, "suspensionLeft");
	
	// Set motor directions
	//frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
	frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
	//backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
	backRight.setDirection(DcMotorSimple.Direction.REVERSE);
	
	// Main Loop
	waitForStart();
	if (opModeIsActive()) {
	  
	  //Set Variables
	  double flp = 0;
	  double frp = 0;
	  double blp = 0;
	  double brp = 0;
	  
	  double speed = 0.7;
	  double turn = 0.55;
	  
	  boolean kaboom = false;
	  
	  boolean intakeOn = false;
	  boolean buttonPress = false;
	  
	  suspensionLeft.setPosition(1);
	  suspensionRight.setPosition(0);
	  
	  
	  while (opModeIsActive()) {
		
		// Arm Movement
		armLeft.setPower((gamepad1.left_trigger * -1)-(gamepad1.right_trigger * -1));
		armRight.setPower((gamepad1.left_trigger * 1)-(gamepad1.right_trigger * 1));
		
		// Suspension
		if (gamepad1.triangle)
		{
			suspensionLeft.setPosition(0.35);
			suspensionRight.setPosition(0.68);
		}
		
		if (gamepad1.dpad_down)
		{
			suspensionLeft.setPosition(1);
			suspensionRight.setPosition(0);
		}
		
		// Intake Toggle
		if (gamepad1.square && buttonPress == false)
		{
			intakeOn = !intakeOn;
			buttonPress = true;
		}
		if (gamepad1.cross)
		{
			intakeTop.setPower(1);
			intakeBottom.setPower(-1);
		}
		else
		{
			if (intakeOn)
			{
				intakeTop.setPower(-0.8);
				intakeBottom.setPower(10);
			}
			else
			{
				intakeTop.setPower(0);
				intakeBottom.setPower(0);
			}
		}
		
		if (gamepad1.square == false)
		{
			buttonPress = false;
		}
		
		// Claw pivoting and placing
		
		if (gamepad1.right_bumper)
		{
			clawPivotLeft.setPosition(0.65);
			clawPivotRight.setPosition(1-0.65);
		}
		else
		{
			clawPivotLeft.setPosition(0.035);
			clawPivotRight.setPosition(1-0.035);
		}
		
		if (gamepad1.left_bumper)
		{
			claw.setPosition(0.7);
		}
		else
		{
			claw.setPosition(1);
		}
		
		
		// Shooter
		if (gamepad1.circle)
		{
		  kaboom = true;
		}
		if (kaboom)
		{
		  shooter.setPosition(0.9);
		} else {
		  shooter.setPosition(0.5);
		}
		
		
		
		// Reset Powers
		flp = 0;
		frp = 0;
		blp = 0;
		brp = 0;
		
		// Rotational Movement
		flp = flp-gamepad1.right_stick_x * turn;
		frp = frp+gamepad1.right_stick_x * turn;
		blp = blp-gamepad1.right_stick_x * turn;
		brp = brp+gamepad1.right_stick_x * turn;
		
		// Vertical Movement
		flp = flp+gamepad1.left_stick_y * speed;
		frp = frp+gamepad1.left_stick_y * speed;
		blp = blp+gamepad1.left_stick_y * speed;
		brp = brp+gamepad1.left_stick_y * speed;
		
		// Horizontal Movement
		flp = (flp - gamepad1.left_stick_x) * speed;
		frp = (frp + gamepad1.left_stick_x) * speed;
		blp = (blp + gamepad1.left_stick_x) * speed;
		brp = (brp - gamepad1.left_stick_x) * speed;
		
		// Set motor powers
		frontLeft.setPower(flp);
		frontRight.setPower(frp);
		backLeft.setPower(blp);
		backRight.setPower(brp);
		
		// Telemetry
		telemetry.addData("Pivot", clawPivotRight.getPosition());
		telemetry.update();
	  }
	}
  }
}
