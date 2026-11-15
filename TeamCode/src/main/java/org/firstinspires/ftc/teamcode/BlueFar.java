package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Blue Far", group = "Mr. Phil")
public class BlueFar extends LinearOpMode
{
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private Servo Pusher;
    private DcMotor Intake;
    private DcMotor Outake2;

    // ===== HOLD TIME VARIABLES =====
    private static final double HOLD_FAST = 0.02;
    private static final double HOLD_SMALL = 0.04;
    private static final double HOLD_ALIGN = 0.25;
    private static final double HOLD_SHOOT = 0.6;

    @Override
    public void runOpMode()
    {
        robot.initialize(true);

        Pusher = hardwareMap.get(Servo.class, "Pusher");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Outake2 = hardwareMap.get(DcMotor.class, "Outake2");
        Outake2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();
        waitForStart();

        robot.resetHeading();  // Reset heading to baseline for Auto

        if (opModeIsActive())
        {
            Pusher.setPosition(0.5);

            // ===== DRIVE FORWARD =====
            robot.drive(68, 1, HOLD_FAST);

            // ===== TURN TO BLUE SIDE =====
            robot.turnTo(46, 0.65, HOLD_ALIGN);

            // ===== SHOOTING SEQUENCE =====
            Outake2.setPower(0.91);
            sleep(2000);

            Intake.setPower(0.75);
            sleep(270);

            Intake.setPower(-0.75);

            sleep(200);

            Intake.setPower(0.65);
            sleep(1500);

            Pusher.setPosition(0);
            sleep((long)(HOLD_SHOOT*1000)); // convert seconds to ms
            Pusher.setPosition(0.5);

            Intake.setPower(0);
            Outake2.setPower(0);

             sleep(1000);
            robot.turnTo(88,0.65, HOLD_ALIGN);
            robot.strafe(48.7,1,0.01);
            Intake.setPower(0.75);
            robot.drive(27,1,0.01);

            Intake.setPower(-0.75);
            sleep(150);
            Intake.setPower(0);


            Outake2.setPower(-0.8);
            sleep(150);
            Outake2.setPower(0);

            robot.drive(-27,1,0.01);
            robot.strafe(-48, 0.65, 0.01);
            Outake2.setPower(0.92);
            robot.turnTo(-0.000002,0.65, HOLD_ALIGN);
            Intake.setPower(0.75);
            sleep(300);
            Intake.setPower(-0.75);
            sleep(200);
            Outake2.setPower(0.95);
            Intake.setPower(0.75);
            sleep(1500);


            Pusher.setPosition(0);
            sleep(600);
            Pusher.setPosition(0.5);
            // Reset encoders & heading
            robot.resetZero();
        }
    }
}
