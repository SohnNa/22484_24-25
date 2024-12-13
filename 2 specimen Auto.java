
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Everything AUTO")
public class Final_Auto_V1 extends LinearOpMode {

  private DcMotorEx back_right;
  private DcMotorEx back_left;
  private DcMotorEx front_right;
  private DcMotorEx front_left;
  private DcMotor arm_main;
  private DcMotor arm_intake;
  //This is the magnetic limit switchs
  TouchSensor mag_arm_lower;// Touch sensor Object
  TouchSensor mag_arm_upper; 

  DistanceSensor distance_1;
  @Override
  public void runOpMode() {
    
 
    
    back_right = hardwareMap.get(DcMotorEx.class, "back_right");
    back_left = hardwareMap.get(DcMotorEx.class, "back_left");
    front_right = hardwareMap.get(DcMotorEx.class, "front_right");
    front_left = hardwareMap.get(DcMotorEx.class, "front_left");
    
    arm_main = hardwareMap.get(DcMotor.class, "arm_main");
    arm_intake = hardwareMap.get(DcMotor.class, "arm_intake");
    Servo servo_intake = hardwareMap.servo.get("servo_intake");
    Servo servo_arm = hardwareMap.servo.get("servo_arm");
    
    distance_1 = hardwareMap.get(DistanceSensor.class, "distance_1");
    
    
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
      //Put run blocks here.
      
      
      
      while (opModeIsActive()) {
        // Put loop blocks here.
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
      Spin();
      moveForward3();
      if (distance_1.getDistance(DistanceUnit.CM) > 10.3 && distance_1.getDistance(DistanceUnit.CM) < 12) {
        servo_arm.setPosition(0);
      }
      sleep(1000);;
      armUp2();
      moveBack();
      Spin();
      strafeLeft();
      smallerForward(); 
      armDown();
      servo_arm.setPosition(1);
      sleep(100);
      armDown2();
      sleep(10000);
                    
                    
            
            
            
            
            
                    
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
    back_right.setPower(0.8);//setVelocity
    back_left.setPower(0.8);//setVelocity
    front_right.setPower(0.8);//setVelocit6
    front_left.setPower(0.8);//setVelocity
    sleep(1000);
    
  }
  
  private void moveForward3() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(789);
    back_right.setTargetPosition(789);
    front_right.setTargetPosition(789);
    back_left.setTargetPosition(789);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.25);//setVelocity
    back_left.setPower(0.25);//setVelocity
    front_right.setPower(0.25);//setVelocity
    front_left.setPower(0.25);//setVelocity
    sleep(1200);
    
  }
  
  
  
 private void smallForward() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //arm_main.setTargetPosition(5000);
    
    
    
    front_left.setTargetPosition(1580);
    back_right.setTargetPosition(1580);
    front_right.setTargetPosition(1580);
    back_left.setTargetPosition(1580);
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
    front_left.setTargetPosition(580);
    back_right.setTargetPosition(580);
    front_right.setTargetPosition(580);
    back_left.setTargetPosition(590);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    sleep(2000);
    
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
    sleep(1500);
  
  }
  private void strafeLeft() {
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setTargetPosition(-2900);
    back_right.setTargetPosition(-2900);
    front_right.setTargetPosition(2900);
    back_left.setTargetPosition(2900);
    front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    sleep(2000);
  
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
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
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
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    sleep(2000);
  
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
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    sleep(2200);
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
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    sleep(1400);
    
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
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    sleep(1800);
  
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
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    sleep(1000);
  
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
    back_right.setPower(0.8);
    back_left.setPower(0.8);
    front_right.setPower(0.8);
    front_left.setPower(0.8);
    sleep(2600);
  
  }
  
  private void armUp() {
    
    arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_main.setTargetPosition(5000);
    
    
    arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    arm_main.setPower(1);
    sleep(1000);
  }
  
  private void armUp2() {
    
    arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_main.setTargetPosition(5000);
    
    
    arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    arm_main.setPower(1);
    sleep(100);
  }
  
    private void armDown() {
    
    arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_main.setTargetPosition(-1850);
    
    
    arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    arm_main.setPower(-1);
    sleep(900);
  }
  
  private void armDown2() {
    
    arm_main.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    arm_main.setTargetPosition(-3230);
    
    
    arm_main.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    arm_main.setPower(-1);
    sleep(1500);
  }
  
  
  
  
  
  
  
  
  
  
}
