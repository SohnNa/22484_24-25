//Plan: 
//moveForward
//strafeRight
//arm extend
//place specimen
//bring back arm
//close claw
//moveBack
//strafeLeft to park

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

@Autonomous(name = "L_1_Auto - 11/17/24")
public class L_1_Auto extends LinearOpMode {

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
      strafeLeft();
      arm_main.setPower(1);
      sleep(1400);
      arm_main.setPower(0);
      sleep(1500);
      smallForward();
      while (opModeIsActive()) {
        // Put loop blocks here.
                    
            if(downAllow){
              arm_main.setPower(-1);
            } else if (upAllow){
                arm_main.setPower(1);
            } else {
              arm_main.setPower(0);
            }
            
            if (mag_arm_upper.isPressed()) {
              //telemetry.addData("Upper touch sensor", "activated");
              upAllow = false;
                  
            } //else {
                //telemetry.addData("Upper Touch Sensor ", "not activated");
            //    upAllow = true;
              
            //}
                
            if (mag_arm_lower.isPressed()) {
              //telemetry.addData("Lower touch sensor", "activated");
              downAllow = false;
            } //else {
                //telemetry.addData("Lower Touch Sensor ", "not activated");
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
    front_left.setTargetPosition(3000);
    back_right.setTargetPosition(3000);
    front_right.setTargetPosition(3000);
    back_left.setTargetPosition(3000);
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
    front_left.setTargetPosition(-3500);
    back_right.setTargetPosition(-3500);
    front_right.setTargetPosition(3500);
    back_left.setTargetPosition(3500);
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
    front_left.setTargetPosition(1600);
    back_right.setTargetPosition(1600);
    front_right.setTargetPosition(-1600);
    back_left.setTargetPosition(-1600);
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
    front_left.setTargetPosition(700);
    back_right.setTargetPosition(700);
    front_right.setTargetPosition(-700);
    back_left.setTargetPosition(-700);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.5);
    back_left.setPower(0.5);
    front_right.setPower(0.5);
    front_left.setPower(0.5);
    sleep(2500);
  
  }
  private void moveBack() {
    int move = -2800;
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
    front_left.setTargetPosition(950);
    back_right.setTargetPosition(950);
    front_right.setTargetPosition(950);
    back_left.setTargetPosition(950);
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
