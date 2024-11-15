//Plan: 
//moveForward
//strafeRight
//arm extend
//place specimen
//bring back arm
//close claw
//moveBack
//strafeRight to park

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "1 Specimen Auto")
public class AutoBottomBlue extends LinearOpMode {

  private DcMotor back_right;
  private DcMotor back_left;
  private DcMotor front_right;
  private DcMotor front_left;
  private DcMotor arm_main;
  private DcMotor arm_intake;
  //This is the magnetic limit switchs
  TouchSensor mag_arm_lower;// Touch sensor Object
  TouchSensor mag_arm_upper; 

  @Override
  public void runOpMode() {
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    arm_main = hardwareMap.get(DcMotor.class, "arm_main");
    arm_intake = hardwareMap.get(DcMotor.class, "arm_intake");
    Servo servo_intake = hardwareMap.servo.get("servo_intake");
    Servo servo_arm = hardwareMap.servo.get("servo_arm");
    
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
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      servo_arm.setPosition(0);
      arm_main.setPower(1);
      sleep(1650);
      arm_main.setPower(0);
      sleep(3000);
      smallForward();
      arm_main.setPower(-1);
      sleep(515);
      arm_main.setPower(0);
      sleep(500);
      servo_arm.setPosition(1);
      sleep(100);
      arm_main.setPower(-1);
      sleep(1135);
      arm_main.setPower(0);
      strafeRight();
      moveBack();
      
      
      
      while (opModeIsActive()) {
        // Put loop blocks here.
                    
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
    front_left.setTargetPosition(2000);
    back_right.setTargetPosition(2000);
    front_right.setTargetPosition(2000);
    back_left.setTargetPosition(2000);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(3000);
    
  }
  
 private void smallForward() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(1550);
    back_right.setTargetPosition(1550);
    front_right.setTargetPosition(1550);
    back_left.setTargetPosition(1550);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(3000);
    
  }
  
  
  
  private void moveLeft() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-1000);
    back_right.setTargetPosition(1000);
    front_right.setTargetPosition(1000);
    back_left.setTargetPosition(-1000);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(3000);
  
  }
  private void strafeLeft() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-1000);
    back_right.setTargetPosition(-1000);
    front_right.setTargetPosition(1000);
    back_left.setTargetPosition(1000);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(3000);
  
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
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(3000);
  
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
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(3000);
  
  }
  private void moveBack() {
    
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-1500);
    back_right.setTargetPosition(-1500);
    front_right.setTargetPosition(-1500);
    back_left.setTargetPosition(-1500);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(3000);
  }
  
  
  
  
  
  
  
  
  
  
}
