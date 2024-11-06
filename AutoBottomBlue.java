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
  private DcMotor arm2;
  private DcMotor arm3;
  //This is the magnetic limit switchs
  TouchSensor lower;// Touch sensor Object
  TouchSensor upper; 

  @Override
  public void runOpMode() {
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    arm2 = hardwareMap.get(DcMotor.class, "arm2");
    arm3 = hardwareMap.get(DcMotor.class, "arm3");
    Servo servo1 = hardwareMap.servo.get("servo1");
    
    //Arm Stuff
    boolean downAllow = false;
    boolean upAllow = false;
    lower = hardwareMap.get(TouchSensor.class, "lower");
    upper = hardwareMap.get(TouchSensor.class, "upper");
    



    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      moveForward();
      upAllow = true;
      sleep(2000);
      smallForward();
      downAllow = true;
      sleep(2000);
      strafeRight();
      moveBack();
      
      while (opModeIsActive()) {
        // Put loop blocks here.
                    
            if(downAllow){
              arm2.setPower(-1);
            } else if (upAllow){
                arm2.setPower(1);
            } else {
              arm2.setPower(1);
            }
            
            if (upper.isPressed()) {
              //telemetry.addData("Upper touch sensor", "activated");
              upAllow = false;
                  
            } //else {
                //telemetry.addData("Upper Touch Sensor ", "not activated");
            //    upAllow = true;
              
            //}
                
            if (lower.isPressed()) {
              //telemetry.addData("Lower touch sensor", "activated");
              downAllow = false;
            } //else {
                //telemetry.addData("Lower Touch Sensor ", "not activated");
             //   downAllow = true;
            //}
    
          
            arm2.setPower(0);
                
                
                
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
    front_left.setTargetPosition(-2000);
    back_right.setTargetPosition(-2000);
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
