package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "GyroTest")
public class GyroTest extends LinearOpMode {

  private IMU imuAsIMU;
  private double robotYaw;
  private double leftPower;
  private double rightPower;
  private double targetRot;
  private double tick;
  private double defaultRot;
  private DcMotor backRight;
  private DcMotor backLeft;
  private DcMotor frontLeft;
  private DcMotor frontRight;
  private double distance_M;
  private DistanceSensor distance;
  private ColorSensor color;
  private double speed;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;
    robotYaw = 0;
    leftPower = 0;
    rightPower = 0;
    targetRot = 0;
    speed = 0.3;
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    
    //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    imuAsIMU = hardwareMap.get(IMU.class, "imu");
    distance = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
    color = hardwareMap.get(ColorSensor.class, "colorSensor");

    // Initialize the IMU.
    // Initializes the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
    imuAsIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
  
    orientation = imuAsIMU.getRobotYawPitchRollAngles();
    angularVelocity = imuAsIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
    robotYaw = orientation.getYaw(AngleUnit.DEGREES);
    
    defaultRot = robotYaw;

    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        telemetry.addData("Yaw", "Press Circle or B on Gamepad to reset.");
        // Check to see if reset yaw is requested.
        if (gamepad1.circle) {
          imuAsIMU.resetYaw();
        }
        // Get the orientation and angular velocity.
        orientation = imuAsIMU.getRobotYawPitchRollAngles();
        angularVelocity = imuAsIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
        robotYaw = orientation.getYaw(AngleUnit.DEGREES);
        
        distance_M = distance.getDistance(DistanceUnit.CM);
        
        //Sets correct power for left and right sides of motors
        if (robotYaw > defaultRot + targetRot){
          leftPower = ((defaultRot + targetRot + robotYaw)/80)+speed;
          rightPower = ((defaultRot + targetRot + robotYaw)/-80)+speed;
        }
        if (robotYaw < targetRot){
          rightPower = ((defaultRot + targetRot + robotYaw)/-80)+speed;
          leftPower = ((defaultRot + targetRot + robotYaw)/80)+speed;
        }
        tick = tick +1;
        
        //if (distance_M < 60){
          //targetRot = targetRot + 15;
        //
        if (targetRot > 180){
          targetRot = -180;
        }
        if (targetRot < -180){
          targetRot = 180;
        }
        if (color.alpha() > 150){
          speed = 0;
        } else{
          speed = 0.3;
        }
        
        // Display yaw, pitch, and roll.
        telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(robotYaw, 2));
        telemetry.addData("Distance", distance_M);
        telemetry.addData("Color", color.argb());
        
        telemetry.addData("R", color.red());
        telemetry.addData("G", color.green());
        telemetry.addData("B", color.blue());
        telemetry.addData("A", color.alpha());
        //telemetry.addData("Right Power", rightPower);
        // Display angular velocity.
        //telemetry.addData("Yaw (Z) velocity", JavaUtil.formatNumber(angularVelocity.zRotationRate, 2));
        //telemetry.addData("Pitch (X) velocity", JavaUtil.formatNumber(angularVelocity.xRotationRate, 2));
        //telemetry.addData("Roll (Y) velocity", JavaUtil.formatNumber(angularVelocity.yRotationRate, 2));
        telemetry.update();
        
        frontLeft.setPower(-leftPower);
        frontRight.setPower(-rightPower);
        backLeft.setPower(-leftPower);
        backRight.setPower(-rightPower);
      }
    }
  }
}

