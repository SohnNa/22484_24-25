
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Encoder_Testing")
public class Encoder_Testing extends LinearOpMode {

  private DcMotorEx back_right;
  private DcMotorEx back_left;
  private DcMotorEx front_right;
  private DcMotorEx front_left;
  private DcMotor arm_main;
  private DcMotor arm_intake;
  private IMU imu;
  //This is the magnetic limit switchs
  TouchSensor mag_arm_lower;// Touch sensor Object
  TouchSensor mag_arm_upper; 
  
  DistanceSensor distance_1;
  @Override
  public void runOpMode() {
    YawPitchRollAngles robotOrentation;
    double current_yaw;
 
    
    
    back_right = hardwareMap.get(DcMotorEx.class, "back_right");
    back_left = hardwareMap.get(DcMotorEx.class, "back_left");
    front_right = hardwareMap.get(DcMotorEx.class, "front_right");
    front_left = hardwareMap.get(DcMotorEx.class, "front_left");
    
    arm_main = hardwareMap.get(DcMotor.class, "arm_main");
    arm_intake = hardwareMap.get(DcMotor.class, "arm_intake");
    Servo servo_intake = hardwareMap.servo.get("servo_intake");
    Servo servo_arm = hardwareMap.servo.get("servo_arm");
    
    distance_1 = hardwareMap.get(DistanceSensor.class, "distance_1");
    
    imu = hardwareMap.get(IMU.class, "imu");
    
    
    
    //Arm Stuff
    boolean downAllow = false;
    boolean upAllow = false;
    
    mag_arm_lower = hardwareMap.get(TouchSensor.class, "mag_arm_lower");
    mag_arm_upper = hardwareMap.get(TouchSensor.class, "mag_arm_upper");
    



    back_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_left.setDirection(DcMotorSimple.Direction.REVERSE);  
    front_right.setDirection(DcMotorSimple.Direction.FORWARD);
    front_left.setDirection(DcMotorSimple.Direction.FORWARD);
    
    arm_main.setDirection(DcMotorSimple.Direction.REVERSE);
    
    
    
    // Put initialization blocks here.
    // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics Control
    // Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction
    // that the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
    imu.resetYaw();
    waitForStart();
    if (opModeIsActive()) {
      //Put run blocks here.
      
      robotOrentation = imu.getRobotYawPitchRollAngles();
      current_yaw = robotOrentation.getYaw(AngleUnit.DEGREES);
      
      
      /*
      servo_arm.setPosition(0);
      armUp();
      smallForward();
      armDown();
      servo_arm.setPosition(1);
      sleep(100);
      armDown2();
      strafeRight2();
      moveForward2();
      moveRight2();
      moveBack2();
      moveForward();
      TurnBy(180);
      Left();
      moveForward3();
      servo_arm.setPosition(0);
      sleep(1000);
      armUp2();
      moveBack();
      TurnBy(180);
      strafeLeft();
      smallerForward(); 
      armDown();
      servo_arm.setPosition(1);
      sleep(100);
      armDown2();
      */
      
      
      
      moveBy(2000);

      //strafeBy(2000);

      //strafeBy(-2000);
      
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.addData("yaw(Z)", JavaUtil.formatNumber(current_yaw, 2));
        telemetry.update();
      
                    
                    
            
            
            
            
            
                    
            //if(downAllow){
            //  arm_main.setPower(-1);
            //} else if (upAllow){
            //    arm_main.setPower(1);
            //} else {
            //  arm_main.setPower(0);
            //}
            
            if (upAllow = false) {
              arm_main.setPower(0);
            }
            
            if (mag_arm_upper.isPressed()) {
              //telemetry.addData("mag_arm_upper touch sensor", "activated");
              upAllow = false;
                  
            } //else {
                //telemetry.addData("mag_arm_upper Touch Sensor ", "not activated");
            //    upAllow = true;
              
            //}
                
            if (mag_arm_lower.isPressed()) {
              //telemetry.addData("mag_arm_lower touch sensor", "activated");
              downAllow = false;
            } //else {
                //telemetry.addData("mag_arm_lower Touch Sensor ", "not activated");
             //   downAllow = true;
            //}
    
          
            arm_main.setPower(0);
                
                
        telemetry.addData("Distance", distance_1.getDistance(DistanceUnit.CM));   
        telemetry.addData("Front_left", front_left.getCurrentPosition());
        telemetry.addData("front_right", front_right.getCurrentPosition());
        telemetry.addData("back_left", back_left.getCurrentPosition());
        telemetry.addData("back_right", back_right.getCurrentPosition());
        telemetry.addData("Arm", arm_main.getCurrentPosition());
        
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void moveForward() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(500);
    back_right.setTargetPosition(500);
    front_right.setTargetPosition(500);
    back_left.setTargetPosition(500);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);//setVelocity
    back_left.setPower(0.6);//setVelocity
    front_right.setPower(0.6);//setVelocit6
    front_left.setPower(0.6);//setVelocity
    while (front_left.getCurrentPosition() < 500) {
      telemetry.addData("Left_Moter", front_left.getCurrentPosition());
      telemetry.update();
    }
    
  }
  
  private void moveForward3() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(790);
    back_right.setTargetPosition(790);
    front_right.setTargetPosition(790);
    back_left.setTargetPosition(790);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.25);//setVelocity
    back_left.setPower(0.25);//setVelocity
    front_right.setPower(0.25);//setVelocity
    front_left.setPower(0.25);//setVelocity
    while (front_left.getCurrentPosition() < 790) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
    
  }
  
  
  
 private void smallForward() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //arm_main.setTargetPosition(5000);
    
    
    
    front_left.setTargetPosition(1540);
    back_right.setTargetPosition(1540);
    front_right.setTargetPosition(1540);
    back_left.setTargetPosition(1540);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    //arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  
    //arm_main.setPower(1);
    
    back_right.setPower(0.7);
    back_left.setPower(0.7);
    front_right.setPower(0.7);
    front_left.setPower(0.7);
    sleep(2000);
    
  }
  
  
   private void smallerForward() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(870);
    back_right.setTargetPosition(870);
    front_right.setTargetPosition(870);
    back_left.setTargetPosition(870);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() < 880) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
    
  }
  
  
  
  
  private void Spin() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-1800);
    back_right.setTargetPosition(1800);
    front_right.setTargetPosition(1800);
    back_left.setTargetPosition(-1800);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.85);
    back_left.setPower(0.85);
    front_right.setPower(0.85);
    front_left.setPower(0.85);
    while (front_left.getCurrentPosition() > -1800) {
      telemetry.addData("Arm", front_left.getCurrentPosition());
      telemetry.update();
    }
  
  }
  private void strafeLeft() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-3200);
    back_right.setTargetPosition(-3200);
    front_right.setTargetPosition(3200);
    back_left.setTargetPosition(3200);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() > -2900) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
  
  }
  
  
  private void Left() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-280);
    back_right.setTargetPosition(-280);
    front_right.setTargetPosition(280);
    back_left.setTargetPosition(280);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() > -280) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
    
    
  }
  
  
  
  private void strafeRight() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(2800);
    back_right.setTargetPosition(2800);
    front_right.setTargetPosition(-2800);
    back_left.setTargetPosition(-2800);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() < 2800) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
  
  }
  private void moveRight() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(1000);
    back_right.setTargetPosition(-1000);
    front_right.setTargetPosition(-1000);
    back_left.setTargetPosition(1000);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    while (front_left.getCurrentPosition() < 1000) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
  
  }
  private void moveBack() {
    
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-800);
    back_right.setTargetPosition(-800);
    front_right.setTargetPosition(-800);
    back_left.setTargetPosition(-800);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() > -800) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
    
    
  }
    private void moveForward2() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(1500);
    back_right.setTargetPosition(1500);
    front_right.setTargetPosition(1500);
    back_left.setTargetPosition(1500);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() < 1500) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
    
  }
  
  private void strafeRight2() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(1600);
    back_right.setTargetPosition(1600);
    front_right.setTargetPosition(-1600);
    back_left.setTargetPosition(-1600);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() < 1600) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
  
  }
  private void moveRight2() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(600);
    back_right.setTargetPosition(600);
    front_right.setTargetPosition(-600);
    back_left.setTargetPosition(-600);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() < 600) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
  
  }
  private void moveBack2() {
    int move = -2500;
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(move);
    back_right.setTargetPosition(move);
    front_right.setTargetPosition(move);
    back_left.setTargetPosition(move);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.6);
    back_left.setPower(0.6);
    front_right.setPower(0.6);
    front_left.setPower(0.6);
    while (front_left.getCurrentPosition() > -2500) {
      telemetry.addData("Left_Motor", front_left.getCurrentPosition());
      telemetry.update();
    }
  
  }
  
  private void armUp() {
    
    arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_main.setTargetPosition(5000);
    
    
    arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    arm_main.setPower(1);
    
    while (arm_main.getCurrentPosition() < 5000) {
      telemetry.addData("Arm", arm_main.getCurrentPosition());
      telemetry.update();
    }

  }
  
  private void armUp2() {
    
    arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_main.setTargetPosition(5000);
    
    
    arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    arm_main.setPower(1);
    while (arm_main.getCurrentPosition() < 5000) {
      telemetry.addData("Arm", arm_main.getCurrentPosition());
      telemetry.update();
    }
  }
  
    private void armDown() {
    
    arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_main.setTargetPosition(-1850);
    
    
    arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    arm_main.setPower(-1);
    while (arm_main.getCurrentPosition() > -1850) {
      telemetry.addData("Arm", arm_main.getCurrentPosition());
      telemetry.update();
    }
  }
  
  private void armDown2() {
    
    arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_main.setTargetPosition(-3250);
    
    
    arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    arm_main.setPower(-1);
    sleep(1500);
  }
  
  //Turning
  
  
  private double Normal_Angle(double angle) {
    while (angle < -180) {
      angle += 360.0;
    }
    
    while (angle > 180) {
      angle -= 360;
    }
    
    return angle;
    
  }
  
  private void TurnBy(double turn) {
    
    double current_yaw, target_yaw, power;
    YawPitchRollAngles robotOrentation;
    
    
    robotOrentation = imu.getRobotYawPitchRollAngles();
    current_yaw = robotOrentation.getYaw(AngleUnit.DEGREES);
    target_yaw = Normal_Angle(current_yaw + turn);
    telemetry.addData("target", JavaUtil.formatNumber(target_yaw, 2));
    telemetry.addData("current", JavaUtil.formatNumber(current_yaw, 2));
    telemetry.update();
    
    back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    
    
    while (opModeIsActive() && Math.abs(Normal_Angle(current_yaw - target_yaw)) > 1 ) {
        
        robotOrentation = imu.getRobotYawPitchRollAngles();
        current_yaw = robotOrentation.getYaw(AngleUnit.DEGREES);

        power = (current_yaw - target_yaw)/100;
        //extend out by .1 and clips it so that it is in the zero to one range. 
        power += 0.1 * Math.signum(power);
        if (power > 0.9) {
          power = 0.9;
        } else if (power < -0.9) {
          power = -0.9;
        } 
        
         
        
        back_right.setPower(-power);
        front_right.setPower(-power);
        back_left.setPower(power);
        front_left.setPower(power);
        telemetry.addData("yaw(Z)", JavaUtil.formatNumber(current_yaw, 2));
        telemetry.addData("Power", power);
        telemetry.update();
      }
      back_right.setPower(0);
      back_left.setPower(0);
      front_left.setPower(0);
      front_right.setPower(0);
    
    
  }
  
  
    private void moveBy(int target) {
      
      double current_yaw, target_yaw, power;
      YawPitchRollAngles robotOrentation;
      
        
      robotOrentation = imu.getRobotYawPitchRollAngles();
      current_yaw = robotOrentation.getYaw(AngleUnit.DEGREES);
      target_yaw = current_yaw;
      double yaw_error = 0;
      double error = 0;

      //signum finds the sum of the number
      
      
 
      double base_power = 0.6 * Math.signum(target);
      
      
      
      back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      front_left.setTargetPosition(target);
      back_right.setTargetPosition(target);
      front_right.setTargetPosition(target);
      back_left.setTargetPosition(target);
      front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      back_right.setPower(base_power);
      back_left.setPower(base_power);
      front_right.setPower(base_power);
      front_left.setPower(base_power);
      while (front_left.getCurrentPosition() < target) {
        current_yaw = robotOrentation.getYaw(AngleUnit.DEGREES);
        yaw_error = current_yaw - target_yaw;
        
        back_right.setPower(base_power - yaw_error/100);
        back_left.setPower(base_power + yaw_error/100);
        front_right.setPower(base_power - yaw_error/100);
        front_left.setPower(base_power + yaw_error/100);
        
        
        telemetry.addData("Left_Moter", front_left.getCurrentPosition());
        telemetry.update();
      }
      //delete 
      sleep(4000);
      error = current_yaw - target_yaw;
      TurnBy(error);
    
  }
  
  
  private void strafeBy(int goal) {
  
      double current_yaw, target_yaw, power;
      YawPitchRollAngles robotOrentation;
      
      robotOrentation = imu.getRobotYawPitchRollAngles();
      current_yaw = robotOrentation.getYaw(AngleUnit.DEGREES);
      target_yaw = current_yaw;
      double yaw_error = 0;
      double error = 0;

      //signum finds the sum of the number


      //I think i do not need the Math.signum becuase the power is positve even if I am going to the left or the right
      //at least thats how it worked on my previous functions.
      double base_power = 0.6 //* Math.signum(goal);
    
      back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      front_left.setTargetPosition(goal);
      back_right.setTargetPosition(goal);
      front_right.setTargetPosition(-goal);
      back_left.setTargetPosition(-goal);
      front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      back_right.setPower(base_power);
      back_left.setPower(base_power);
      front_right.setPower(base_power);
      front_left.setPower(base_power);

      //When the robot has not finished its movement. 
      while (front_left.getCurrentPosition() < goal) {
        
        current_yaw = robotOrentation.getYaw(AngleUnit.DEGREES);
        yaw_error = current_yaw - target_yaw;
        //The signs may need to be switched. Will know if robot starts spinning or something of that sort. 
        //IF/when the robot becomes unaligned, the wheels will compensate by having some spin slower. 
        back_right.setPower(base_power - yaw_error/100);
        back_left.setPower(base_power + yaw_error/100);
        front_right.setPower(base_power - yaw_error/100);
        front_left.setPower(base_power + yaw_error/100);
      
        telemetry.addData("Left_Motor", front_left.getCurrentPosition());
        telemetry.update();
      }
      //Checks at the end of movement so that the robot is still aligned correctly. 
      //delete
      sleep(4000);
      error = current_yaw - target_yaw;
      TurnBy(error);

  }
  
  
  
  
  
  
  
  
  
  
  
  
}
