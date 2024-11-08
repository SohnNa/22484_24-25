

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



@Autonomous(name = "Follow_Auto")

public class FieldCentricMecanumTeleOp_ extends SynchronousOpMode {
    //https://www.youtube.com/watch?v=kOapppDNlSA
    //This is the magnetic limit switchs
    TouchSensor lower;  // Touch sensor Object
    TouchSensor upper; 
    DcMotor arm3;
    DcMotor arm2;
     double speed = 1.0f; 
        boolean test = false; 
        boolean test1 = true;
        String test2 = "Offline";
        //notMovingUp fixes the butterfinger problem we found during the first comp.
        boolean notMovingUp = true;
        arm3 = hardwareMap.get(DcMotor.class, "arm3");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        lower = hardwareMap.get(TouchSensor.class, "upper");
        upper = hardwareMap.get(TouchSensor.class, "lower");
        boolean downAllow = true;
        boolean upAllow = true;
        // THE DECLARING of the... MOTORS!!!!!
        DcMotor front_left = hardwareMap.dcMotor.get("front_left");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left");
        DcMotor front_right = hardwareMap.dcMotor.get("front_right");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right");
        Servo servo1 = hardwareMap.servo.get("servo1");
        Servo servo2 = hardwareMap.servo.get("servo2");
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
    @Override public void runOpMode() throws InterruptedException {
    
