package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class FollowAprilTag extends LinearOpMode {

    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Servo ClawRight;
    private Servo ClawLeft;

    @Override
    public void runOpMode() {
        
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        double flp = 0;
        double frp = 0;
        double blp = 0;
        double brp = 0;
        
        double tflp = 0;
        double tfrp = 0;
        double tblp = 0;
        double tbrp = 0;
      
        double speed = 0;
        double turn = 0.12;

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();
        
        VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new android.util.Size(640, 480))
            .build();
        
        waitForStart();
        
        while (!isStopRequested() && opModeIsActive()) {
            
            if (tagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                
                //Rotational Movement
                tflp = (flp - (tag.ftcPose.x * 1)) * turn;
                tfrp = (frp + (tag.ftcPose.x * -1)) * turn;
                tblp = (blp - (tag.ftcPose.x * -1)) * turn;
                tbrp = (brp + (tag.ftcPose.x * -1)) * turn;
                
                //Vertical Movement
                //tflp = flp - speed;
                //tfrp = frp + speed;
                //tblp = blp + speed;
                //tbrp = brp + speed;
                
                //Smoothing
                flp = flp+((tflp-flp)/7);
                frp = frp+((tfrp-frp)/7);
                blp = blp+((tblp-blp)/7);
                brp = brp+((tbrp-brp)/7);
                
                
                //Set motor powers
                frontLeft.setPower(flp);
                frontRight.setPower(frp);
                backLeft.setPower(blp);
                backRight.setPower(brp);
                
            }
            else {
                flp = 0;
                frp = 0;
                blp = 0;
                brp = 0;
                
                frontLeft.setPower(flp);
                frontRight.setPower(frp);
                backLeft.setPower(blp);
                backRight.setPower(brp);
                
            }
        }
        
    }
}
