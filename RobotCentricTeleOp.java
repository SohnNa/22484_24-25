package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Disabled
@TeleOp (name = "'Better' Field Centric Driving - 10/25/24 (THIS IS ROBOT CENTRIC IN CASE YOU WERE WONDERING ROMAN)")
public class OtherFieldCentricMecanumTeleOp extends LinearOpMode {
        TouchSensor lower;  // Touch sensor Object
        TouchSensor upper; 
        DcMotor arm3;
        DcMotor arm2;
        @Override
    public void runOpMode() throws InterruptedException {
        
        double speed = 1.0f;
        notMovingUp = true;
        
        
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor front_left = hardwareMap.dcMotor.get("front_left");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left");
        DcMotor front_right = hardwareMap.dcMotor.get("front_right");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right");
        Servo servo1 = hardwareMap.servo.get("servo1");
        Servo servo2 = hardwareMap.servo.get("servo2");
        
        
        boolean test = false; 
        boolean test1 = true;
        String test2 = "Offline";
        arm3 = hardwareMap.get(DcMotor.class, "arm3");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        lower = hardwareMap.get(TouchSensor.class, "upper");
        upper = hardwareMap.get(TouchSensor.class, "lower");
        boolean downAllow = true;
        boolean upAllow = true;




        // Reversing the motors so that the wheels spin forward when the robot is told to move forward. 
        // As opposed to turning when told to go forward. 

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double front_left_Power = (y + x + rx) / denominator;
            double back_left_Power = (y - x + rx) / denominator;
            double front_right_Power = (y - x - rx) / denominator;
            double back_right_Power = (y + x - rx) / denominator;
            
            
            if(gamepad2.dpad_down && downAllow){
                    arm3.setPower(-1);
                } if (gamepad2.dpad_up && upAllow){
                    arm3.setPower(1);
                    notMovingUp = false;
                }
            notMovingUp = true;    


            //controls for the secound arm. will  be controled with the y and a buttons. 
            //reason there is no checks is becuase we are not puttiong on magnetic limit switchs for some reason...

            if (gamepad2.a) {
                arm2.setPower(0.5);
            }

            if (gamepad2.y) {
                arm2.setPower(-0.5);
            }
                
                // lower is the bottom magnetic limit. Upper is the top magnetic limit switch.
                
                //This is setting it so that when a magnetic limit switch is activated, it changes the Vairable
                //upAllow from true to false or vice-versa
                if (upper.isPressed()) {
                //telemetry.addData("Upper touch sensor", "activated");
                upAllow = false
                notMovingUp = true;  
                
                } else {
                    //telemetry.addData("Upper Touch Sensor ", "not activated");
                    upAllow = true;
              
                }
                
                //This is setting it so that when a magnetic limit switch is activated, it changes the Vairable
                //downAllow from true to false or vice-versa/ 
                if (lower.isPressed()) {
                    
                    //telemetry.addData("Lower touch sensor", "activated");
                    downAllow = false;
                } else {
                    //telemetry.addData("Lower Touch Sensor ", "not activated");
                    downAllow = true;
                }
    
          
                arm3.setPower(0);
                arm2.setPower(0);
                
                
                
            
            
            
            
            
            
            
            
            if (gamepad1.a) {
                speed = 1.0f;
            }
            
            if (gamepad1.b) {
                speed = 0.75f; 
            } 
            
            if (gamepad1.y) {
                
                speed = 0.5f; 
            }
            
            if (gamepad1.x) {
                speed = 0.25f;
            }
            
            
            
            
            if (gamepad2.dpad_left && notMovingUp) {
                servo1.setPosition(1);
            }
            
            if (gamepad2.dpad_right && notMovingUp) {
                servo1.setPosition(0);
            }


            if (gamepad2.x) {
                servo2.setPosition(1);

            } else if (gamepad2.b) {
                servo2.setPosition(0);
            }
            
            
            
            
            telemetry.addData("Switch", test2);
            telemetry.addData("Speed", speed);
            telemetry.update();
            
            
            
            
            
            
            front_left.setPower(front_left_Power * speed);
            back_left.setPower(back_left_Power * speed);
            front_right.setPower(front_right_Power * speed);
            back_right.setPower(back_right_Power * speed);
        }
    }
}
