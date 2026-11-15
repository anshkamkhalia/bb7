/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SimplifiedOdometryRobot;
/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Right BB7", group = "Mr. Phil")

public class RightAuto extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private Servo Pusher;
    private Servo Belt;
    private DcMotor Intake;
    private DcMotor Outake1;
    private DcMotor Outake2;
    private boolean test;


    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);
        {

//        Belt = hardwareMap.get(Servo.class, "Belt");
//        Belt.setDirection(Servo.Direction.REVERSE);
//        Pusher = hardwareMap.get(Servo.class, "Pusher");
//        Pusher.setDirection(Servo.Direction.REVERSE);
//        Outake1 =hardwareMap.get(DcMotor.class, "Outake1");
//        Outake2 = hardwareMap.get(DcMotor.class, "Outake2");
//        Intake = hardwareMap.get(DcMotor.class, "Intake");
            Intake = hardwareMap.get(DcMotor.class, "Intake");
            Outake2 = hardwareMap.get(DcMotor.class, "Outake2");
            Outake2.setDirection(DcMotor.Direction.REVERSE);
            //robot.resetZero();
            // Wait for driver to press start
            telemetry.addData(">", "Touch Play to run Auto");
            telemetry.update();
        }
        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto
        //robot.resetZero();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            //robot.turnTo(-43,1,0.25);
            //robot.strafe(  65, 1, 0.25);
            //robot.drive(10,1,0.25);
            //Outake2.setPower(0.7);
            //sleep(2000);
            //robot.drive(-10,1,0.25);
            //Intake. setPower(-1);
            //sleep(750);
            //robot.drive(5,1,0.25);
            //sleep(500);
            //Intake.setPower(0);
            //sleep(400);
            //robot.drive(-10,1,0.25);
            //Intake.setPower(-1);
            //sleep(400);
            //Intake.setPower(0);
            //robot.drive(5,1,0.25);
            //sleep(1000);
            //robot.drive(-10,1,0.25);
            //Intake.setPower(-1);
            //sleep(400);
            //Intake.setPower(0);
            //Outake2.setPower(0);
            //robot.resetZero();
        }
    }
}
