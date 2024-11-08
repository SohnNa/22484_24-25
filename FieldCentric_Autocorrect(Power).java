

//PLEASE MAKE SURE THE OWNER OF THIS CODE IS AWARE OF ANY CHANGES YOU ARE MAKING AND
//UNDERSTANDS HOW THE CODE YOU ARE ADDING WORKS. And please, do not delete anything. 
//If some already existing code is causing problems, write the code on a different gamepad
//or create a new code. 






package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


//front_left = port 0
//front_right = port 1
//back_left = port 2
//back_right = port 3
//A note to future selves. 




@TeleOp(name = "Fixing Field Centric - 11/6/24 (Power)")
//@Disabled
public class FieldCentricMecanumTeleOp_ extends LinearOpMode {
    
    //This is the magnetic limit switchs
    TouchSensor mag_arm_lower;  // Touch sensor Object
    TouchSensor mag_arm_upper; 
    DcMotor arm_main;
    DcMotor arm_intake;
    @Override
    public void runOpMode() throws InterruptedException {
        
        
        double speed = 1.0f; 
        boolean test = false; 
        boolean test1 = true;
        String test2 = "Offline";
        //notMovingUp fixes the butterfinger problem we found during the first comp.
        boolean notMovingUp = true;
        arm_main = hardwareMap.get(DcMotor.class, "arm_main");
        arm_intake = hardwareMap.get(DcMotor.class, "arm_intake");
        mag_arm_lower = hardwareMap.get(TouchSensor.class, "mag_arm_lower");
        mag_arm_upper = hardwareMap.get(TouchSensor.class, "mag_arm_upper");
        boolean downAllow = true;
        boolean upAllow = true;
        
        
        
        // THE DECLARING of the... MOTORS!!!!!
        DcMotor front_left = hardwareMap.dcMotor.get("front_left");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left");
        DcMotor front_right = hardwareMap.dcMotor.get("front_right");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right");
        Servo servo_intake = hardwareMap.servo.get("servo_intake");
        Servo servo_arm = hardwareMap.servo.get("servo_arm");

        
        // Reversing the motors. If you want an explanation, see Robot Centric Code. 
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        

        waitForStart();
        imu.resetYaw();
        //This finds the desired way the robot should point at the start...yeah
        double desiredBotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // might change the direction that is forward when driving in field centric. When tested, it worked but only when the self-correcting was not added. 
            //This data is correct as of 10/9/24

           // if (gamepad1.left_bumper) {
           //     imu.resetYaw();
           // }
            
            
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
           
           
           
            if (rx != 0) {
                desiredBotHeading = botHeading; 
            }
            else {
                rx = (botHeading + desiredBotHeading)/(Math.PI/2);
            if (rx > 1) {
                rx = 1;
            }
            else if (rx < -1) {
                rx = -1; 
            }

            }
            
            
            
            
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double front_left_Power = (rotY + rotX + rx) / denominator;
            double back_left_Power = (rotY - rotX + rx) / denominator;
            double front_right_Power = (rotY - rotX - rx) / denominator;
            double back_right_Power = (rotY + rotX - rx) / denominator;


            //if my understanding is correct, this makes it so that the arm can only go up or down when
            //the corrasponding Variable is set to true. I swaped it to gamepad 2 so that it does not
            //give one driver too much to do at the same time. It is controled with up and down on the dpad. 
            
            if(gamepad2.dpad_down && downAllow){
                    arm_main.setPower(-1);
                } else if (gamepad2.dpad_up && upAllow){
                    arm_main.setPower(1);
                    notMovingUp = false;
                } else {
                    notMovingUp = true;
                }
            
            
            


            //controls for the secound arm. will  be controled with the y and a buttons. 
            //reason there is no checks is becuase we are not puttiong on magnetic limit switchs for some reason...

            if (gamepad2.y) {
                arm_intake.setPower(1);
            }

            if (gamepad2.a) {
                arm_intake.setPower(-1);
            }
                
                // lower is the bottom magnetic limit. Upper is the top magnetic limit switch.
                
                //This is setting it so that when a magnetic limit switch is activated, it changes the Vairable
                //upAllow from true to false or vice-versa
                if (mag_arm_upper.isPressed()) {
                    telemetry.addData("mag_arm_upper touch sensor", "activated");
                    upAllow = false;
                    notMovingUp = true;
                  
                } else {
                    telemetry.addData("mag_arm_upper Touch Sensor ", "not activated");
                    upAllow = true;
              
                }
                
                //This is setting it so that when a magnetic limit switch is activated, it changes the Vairable
                //downAllow from true to false or vice-versa/ 
                if (mag_arm_lower.isPressed()) {
                    
                    telemetry.addData("mag_arm_lower touch sensor", "activated");
                    downAllow = false;
                } else {
                    telemetry.addData("mag_arm_lower Touch Sensor ", "not activated");
                    downAllow = true;
                }
    
          
                arm_main.setPower(0);
                arm_intake.setPower(0);
                
                
                
            //Setting "Gears" to the robot so that the driver can slow the robot down without
            //risk of misusing or damaging the controller. Getting tired of re-implamenting this
            
            
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

            //makeing it so that the servo can open and close. 
            //Left is open, Right is closed
            //dpad controls end effector that goes up, x and b for end effector that extends out.  
            if (gamepad2.dpad_left && notMovingUp) {
                servo_arm.setPosition(1);
            }
            
            if (gamepad2.dpad_right) {
                servo_arm.setPosition(0);
            }


            if (gamepad2.x) {
                servo_intake.setPosition(1);

            } else if (gamepad2.b) {
                servo_intake.setPosition(0);
            }
            
            
            //This does work...Makes the robot be stuck between 100% and 50%. Locks out the 
            //other buttons. Not sure if I want to keep this. 10/18/24
            
            if (gamepad1.right_bumper || test) {
                speed = 0.5f;
                test2 = "On";
            } else {
                speed = 1.0f;
                test2 = "Off";
                test = false;
            }
           
          
            telemetry.addData("Switch", test2);
            telemetry.addData("Speed", speed);
            telemetry.addData("notMovingUp", notMovingUp);
            telemetry.update();
                
            
            
            //Total power calculations. 
            
            front_left.setPower(front_left_Power * speed);
            back_left.setPower(back_left_Power * speed);
            front_right.setPower(front_right_Power * speed);
            back_right.setPower(back_right_Power * speed);
        }
    }
}
