package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoTesla", group = "Linear Opmode")
@Disabled
public class AutoTesla extends LinearOpMode {

    // Hardware map
    private DcMotor leftEncoder, rightEncoder, centerEncoder;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Robot odometry variables
    private double robotX = 0; // X position in inches
    private double robotY = 0; // Y position in inches
    private double robotAngle = 0; // Angle in radians

    // Constants
    private static final double TICKS_PER_REV = 2000; // Change based on your encoders
    private static final double WHEEL_DIAMETER = 2.0; // Wheel diameter in inches
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
    private static final double TRACK_WIDTH = 4.0; // Distance between left and right encoders in inches
    private static final double CENTER_WHEEL_OFFSET = 7.0; // Offset of the center encoder

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftEncoder = hardwareMap.get(DcMotor.class, "m1");
        //rightEncoder = hardwareMap.get(DcMotor.class, "m4");
        centerEncoder = hardwareMap.get(DcMotor.class, "m4");
        
        frontLeft = hardwareMap.get(DcMotor.class, "m4");
        frontRight = hardwareMap.get(DcMotor.class, "m3");
        backLeft = hardwareMap.get(DcMotor.class, "m2");
        backRight = hardwareMap.get(DcMotor.class, "m1");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            updateOdometry();

            // Example: Drive to position (24, 24) in inches
            driveToPosition(6, 6, 0);

            // Stop the robot
            stopMotors();
            break;
        }
    }

    private void updateOdometry() {
        double leftPosition = leftEncoder.getCurrentPosition() / TICKS_PER_INCH;
        //double rightPosition = rightEncoder.getCurrentPosition() / TICKS_PER_INCH;
        double centerPosition = centerEncoder.getCurrentPosition() / TICKS_PER_INCH;

        double deltaLeft = leftPosition;
        //double deltaRight = rightPosition;
        double deltaCenter = centerPosition;

        double deltaAngle = ( deltaLeft) / TRACK_WIDTH;
        double deltaX = deltaCenter - (CENTER_WHEEL_OFFSET * deltaAngle);

        robotAngle += deltaAngle;
        robotX += deltaX * Math.cos(robotAngle);
        robotY += deltaX * Math.sin(robotAngle);

        telemetry.addData("X", robotX);
        telemetry.addData("Y", robotY);
        telemetry.addData("Angle (rad)", robotAngle);
        telemetry.update();
    }

    private void driveToPosition(double targetX, double targetY, double power) {
        double errorX = targetX - robotX;
        double errorY = targetY - robotY;
        double distance = Math.hypot(errorX, errorY);
        double angle = Math.atan2(errorY, errorX) - robotAngle;

        double drivePower = power;
        double turnPower = angle;

        while (opModeIsActive() && distance > 1) {
            errorX = targetX - robotX;
            errorY = targetY - robotY;
            distance = Math.hypot(errorX, errorY);
            angle = Math.atan2(errorY, errorX) - robotAngle;

            double driveX = drivePower * Math.cos(angle);
            double driveY = drivePower * Math.sin(angle);
            double turn = turnPower;

            setMotorPowers(driveY + turn, driveX - turn, driveY - turn, driveX + turn);

            updateOdometry();
        }
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
}