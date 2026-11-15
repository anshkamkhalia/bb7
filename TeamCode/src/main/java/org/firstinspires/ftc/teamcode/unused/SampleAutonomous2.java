/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Right Auto odom", group = "Mr. Phil")
@Disabled
public class SampleAutonomous2 extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor OutakeRotation;
    private Servo OutakeClaw1;
    private Servo OutakeClaw2;
    private DcMotor LA;
    private DcMotor OutakeSlide;

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);

        OutakeClaw1 = hardwareMap.get(Servo.class, "OutakeClaw1");
        OutakeClaw2 = hardwareMap.get(Servo.class, "OutakeClaw2");
        OutakeRotation = hardwareMap.get(DcMotor.class, "OutakeRotation");
        OutakeClaw2.setDirection(Servo.Direction.REVERSE);
        OutakeSlide =hardwareMap.get(DcMotor.class, "OutakeSlide");
        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();
        LA = hardwareMap.get(DcMotor.class, "LA");


        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {

            OutakeClaw1.setPosition(1);
            OutakeClaw2.setPosition(1);
            sleep(500);
            robot.strafe(  15, 1, 0.25);
            robot.drive(  29, 1, 0.25);
            sleep(500);
            OutakeRotation.setPower(1);
            sleep(3200);
            OutakeRotation.setPower(0);
            sleep(500);;
            OutakeClaw1.setPosition(0);
            OutakeClaw2.setPosition(0);
            robot.strafe( -5, 1, 0.15);
            OutakeClaw1.setPosition(0.5);
            OutakeClaw2.setPosition(0.5);
            OutakeRotation.setPower(-1);
            sleep(3200);
            OutakeRotation.setPower(0);
            OutakeClaw1.setPosition(0);
            OutakeClaw2.setPosition(0);
            robot.drive(  -25, 1, 0.25);
            robot.strafe( -49, 1, 0.15);
            robot.turnTo(  180, 1, 0.15);
            robot.drive(  6, 1, 0.25);
            LA.setPower(1);
            sleep(9000);
            LA.setPower(0);
        }
    }
}
