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

@Autonomous(name = "AutoBottomBlue")
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
    



    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      servo_arm.setPosition(1);
      moveForward();
      upAllow = true;
      sleep(3000);
      smallForward();
      downAllow = true;
      sleep(1000);
      downAllow = false;
      servo_arm.setPosition(0);
      downAllow = true;
      sleep(3000);
      strafeRight();
      servo_arm.setPosition(1);
      sleep(1000);
      moveBack();
      
      while (opModeIsActive()) {
        // Put loop blocks here.
                    
            if(downAllow){
              arm_main.setPower(-1);
            } else if (upAllow){
                arm_main.setPower(1);
            } else {
              arm_main.setPower(1);
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
    front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    sleep(5000);
    
  }
  
  private void smallForward() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    front_left.setTargetPosition(1000);
    back_right.setTargetPosition(1000);
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
    sleep(5000);
    
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
    sleep(5000);
  
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
    sleep(5000);
  
  }
  
  
  private void strafeRight() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-2500);
    back_right.setTargetPosition(-2500);
    front_right.setTargetPosition(2500);
    back_left.setTargetPosition(2500);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(5000);
  
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
    sleep(5000);
  
  }
  private void moveBack() {
    
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-2000);
    back_right.setTargetPosition(-2000);
    front_right.setTargetPosition(-2000);
    back_left.setTargetPosition(-2000);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(5000);
  }
  
  
  
  
  
  
  
  
  
  
}
