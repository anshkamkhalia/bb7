package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto Code 1")
@Disabled
public class AutoRight extends LinearOpMode {
    // Hardware variables for motors and encoders
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftEncoder, rightEncoder, perpendicularEncoder;
    private DcMotor OutakeRotation;
    private Servo OutakeClaw1;
    private Servo OutakeClaw2;

    // Constants for odometry calculations
    private static final double COUNTS_PER_REV = 2000;  // Encoder ticks per revolution
    private static final double WHEEL_DIAMETER = 1.25984;   // Diameter of odometry wheel (in inches)
    private static final double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    private static final double TRACK_WIDTH = 11.0;     // Distance between the two odometry wheels (in inches)
private static final double PERPENDICULAR_OFFSET = 6.0; // Distance of perpendicular encoder from robot center (in inches)


    // Robot's position on the field
    private double xPos = 0.0;  // X position in inches
    private double yPos = 0.0;  // Y position in inches
    private double heading = 0.0; // Robot heading in radians

    // Previous encoder values
    private int prevLeftEncoder = 0;
    private int prevRightEncoder = 0;
    private int prevPerpendicularEncoder = 0;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "m4");
        frontRight = hardwareMap.get(DcMotor.class, "m3");
        backLeft = hardwareMap.get(DcMotor.class, "m2");
        backRight = hardwareMap.get(DcMotor.class, "m1");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        OutakeClaw1 = hardwareMap.get(Servo.class, "OutakeClaw1");
        OutakeClaw2 = hardwareMap.get(Servo.class, "OutakeClaw2");
        OutakeRotation = hardwareMap.get(DcMotor.class, "OutakeRotation");
        OutakeClaw2.setDirection(Servo.Direction.REVERSE);

        leftEncoder = hardwareMap.get(DcMotor.class, "m2"); //x
        rightEncoder = hardwareMap.get(DcMotor.class, "m1");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "m4"); //y
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);
        // Reset encoders
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Odometry initialized. Waiting for start...");
        telemetry.update();


        waitForStart();
        if ( opModeIsActive()) {
            OutakeClaw1.setPosition(1);
            OutakeClaw2.setPosition(1);
            sleep(1000);
            if (perpendicularEncoder.getCurrentPosition() <14500 ) {
                right();
            }
            else if (perpendicularEncoder.getCurrentPosition() >= 14500) {
                motorStop();
                perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

//            if (leftEncoder.getCurrentPosition()<15600) {
//                forward();
//            }
//            else if (leftEncoder.getCurrentPosition() >=15600) {
//                motorStop();
//                leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            }
        }



    }
    private void updateOdometry() {
        int currentLeftEncoder = leftEncoder.getCurrentPosition();
        int currentRightEncoder = rightEncoder.getCurrentPosition();
        int currentPerpendicularEncoder = perpendicularEncoder.getCurrentPosition();

        // Calculate changes in encoder values
        int deltaLeft = currentLeftEncoder - prevLeftEncoder;
        int deltaRight = currentRightEncoder - prevRightEncoder;
        int deltaPerpendicular = currentPerpendicularEncoder - prevPerpendicularEncoder;

        // Update previous encoder values
        prevLeftEncoder = currentLeftEncoder;
        prevRightEncoder = currentRightEncoder;
        prevPerpendicularEncoder = currentPerpendicularEncoder;

        // Calculate distance moved by each wheel
        double deltaLeftInches = deltaLeft / COUNTS_PER_INCH;
        double deltaRightInches = deltaRight / COUNTS_PER_INCH;
        double deltaPerpendicularInches = deltaPerpendicular / COUNTS_PER_INCH;

        // Calculate the change in heading
        double deltaHeading = (deltaRightInches - deltaLeftInches) / TRACK_WIDTH;

        // Average forward movement
        double deltaForward = (deltaLeftInches + deltaRightInches) / 2.0;

        // Adjust for heading changes
        double deltaX, deltaY;

        if (deltaHeading != 0) {
            double radius = deltaForward / deltaHeading;

            // Calculate local movement
            deltaX = radius * Math.sin(deltaHeading);
            deltaY = radius * (1 - Math.cos(deltaHeading)) + deltaPerpendicularInches;

        } else {
            // Robot moved in a straight line
            deltaX = deltaForward;
            deltaY = deltaPerpendicularInches;
        }

        // Convert to global field coordinates
        double cosHeading = Math.cos(heading);
        double sinHeading = Math.sin(heading);

        double globalDeltaX = deltaX * cosHeading - deltaY * sinHeading;
        double globalDeltaY = deltaX * sinHeading + deltaY * cosHeading;

        // Update robot's position and heading
        xPos += globalDeltaX;
        yPos += globalDeltaY;
        heading += deltaHeading;

        // Wrap heading to [-pi, pi]
        heading = ((heading + Math.PI) % (2 * Math.PI)) - Math.PI;

        // Display odometry data
        telemetry.addData("X Position (in)", xPos);
        telemetry.addData("Y Position (in)", yPos);
        telemetry.addData("Heading (rad)", heading);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("M2 currentLeftEncoder ", currentLeftEncoder);
        telemetry.addData("M1 currentRightEncoder ", currentRightEncoder);
        telemetry.addData("M4 currentPerpendicularEncoder ", currentPerpendicularEncoder);
        telemetry.update();
    }

    private void driveToPosition(double targetX, double targetY, double targetHeading) {
        // Calculate errors
        updateOdometry();
        double xError = targetX - xPos;
        double yError = targetY - yPos;
        double distance = Math.hypot(xError, yError);
        while (opModeIsActive() && distance > 1) {
            xError = targetX - xPos;
            yError = targetY - yPos;
            distance = Math.hypot(xError, yError);
            double headingError = targetHeading - heading;

            // Normalize heading error to [-pi, pi]
            headingError = ((headingError + Math.PI) % (2 * Math.PI)) - Math.PI;

            // Proportional control constants
            double kP = 0.05; // Position control constant
            double kH = 0.1;  // Heading control constant

            // Calculate control outputs
            double forwardPower = kP * yError;
            double lateralPower = kP * xError;
            double turnPower = kH * headingError;

            // Calculate motor powers
            double flPower = forwardPower + lateralPower + turnPower;
            double frPower = forwardPower - lateralPower - turnPower;
            double blPower = forwardPower - lateralPower + turnPower;
            double brPower = forwardPower + lateralPower - turnPower;

            // Normalize motor powers
            double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower)));
            if (maxPower > 1.0) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
            updateOdometry();
        }

    }
    private void motorStop () {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        updateOdometry();

    }
    private void right () {
        double targetX = 6; // Target X position in inches
        double targetY = 0; // Target Y position in inches
        double targetHeading = 0; // Target heading in radians
        // Drive to the target position
        driveToPosition(targetX, targetY, targetHeading);
        // Set motor power
    }
    private void forward () {
        double targetX = 0; // Target X position in inches
        double targetY = 6; // Target Y position in inches
        double targetHeading = 0.0; // Target heading in radians
        // Drive to the target position
        driveToPosition(targetX, targetY, targetHeading);
    }
    private void back () {
        double targetX = 0; // Target X position in inches
        double targetY = -6; // Target Y position in inches
        double targetHeading = 0.0; // Target heading in radians
        // Drive to the target position
        driveToPosition(targetX, targetY, targetHeading);
    }
    private void sideabit () {
        double targetX = 6; // Target X position in inches
        double targetY = 0; // Target Y position in inches
        double targetHeading = 0.0; // Target heading in radians
        // Drive to the target position
        driveToPosition(targetX, targetY, targetHeading);
    }

    // Check if the robot has reached the target position
    private boolean reachedTarget(double targetX, double targetY) {
        return (Math.abs(targetX - xPos) < 0.5) && (Math.abs(targetY - yPos) < 0.5);
    }
}

