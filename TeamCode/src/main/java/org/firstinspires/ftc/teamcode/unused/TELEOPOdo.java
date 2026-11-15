package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drive with 2-Wheel Odometry2345", group = "Example")
@Disabled
public class TELEOPOdo extends LinearOpMode {
    // Hardware variables for motors and encoders
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor forwardEncoder, lateralEncoder;

    // Constants for odometry calculations
    private static final double COUNTS_PER_REV = 2000;  // Encoder ticks per revolution
    private static final double WHEEL_DIAMETER = 2.0;   // Diameter of odometry wheel (in inches)
    private static final double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    private static final double TRACK_WIDTH = 4;     // Distance between the two odometry wheels (in inches)

    // Robot's position on the field
    private double xPos = 0.0;  // X position in inches
    private double yPos = 0.0;  // Y position in inches
    private double heading = 0.0; // Robot heading in radians

    // Previous encoder values
    private int prevForwardEncoder = 0;
    private int prevLateralEncoder = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "m4");
        frontRight = hardwareMap.get(DcMotor.class, "m3");
        backLeft = hardwareMap.get(DcMotor.class, "m2");
        backRight = hardwareMap.get(DcMotor.class, "m1");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        forwardEncoder = hardwareMap.get(DcMotor.class, "m4"); //y
        lateralEncoder = hardwareMap.get(DcMotor.class, "m1"); //x

        // Reset encoders
        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forwardEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lateralEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Odometry initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            // Update the robot's position
            updateOdometry();
            if (gamepad1.x) {
                // Example: Move to target position (24, 24) with a target heading of 0
                double targetX = 100; // Target X position in inches
                double targetY = 0; // Target Y position in inches
                double targetHeading = 0.0; // Target heading in radians
                // Drive to the target position
                driveToPosition(targetX, targetY, targetHeading);
            }
            if (gamepad1.a) {
                // Example: Move to target position (24, 24) with a target heading of 0
                double targetX = 0; // Target X position in inches
                double targetY = 100; // Target Y position in inches
                double targetHeading = 0.0; // Target heading in radians
                // Drive to the target position
                driveToPosition(targetX, targetY, targetHeading);
                motorStop();

            }
            if (gamepad1.b) {
                // Example: Move to target position (24, 24) with a target heading of 0
                double targetX = 50; // Target X position in inches
                double targetY = 50; // Target Y position in inches
                double targetHeading = 15.0; // Target heading in radians
                // Drive to the target position
                driveToPosition(targetX, targetY, targetHeading);
                motorStop();

            }
            if (gamepad1.y){
                motorStop();
            }
        }
    }

    private void updateOdometry() {
        // Get current encoder readings
        int currentForwardEncoder = forwardEncoder.getCurrentPosition();
        int currentLateralEncoder = lateralEncoder.getCurrentPosition();

        // Calculate changes in encoder values
        int deltaForward = currentForwardEncoder - prevForwardEncoder;
        int deltaLateral = currentLateralEncoder - prevLateralEncoder;

        // Update previous encoder values
        prevForwardEncoder = currentForwardEncoder;
        prevLateralEncoder = currentLateralEncoder;

        // Calculate distances moved
        double deltaForwardInches = deltaForward / COUNTS_PER_INCH;
        double deltaLateralInches = deltaLateral / COUNTS_PER_INCH;

        // Calculate heading (assume lateral wheel tracks rotation)
        double deltaHeading = deltaLateralInches / TRACK_WIDTH;

        // Adjust for heading changes
        double deltaX, deltaY;
        if (deltaHeading != 0) {
            // Robot moved in an arc
            double radius = deltaForwardInches / deltaHeading;
            deltaX = radius * (1 - Math.cos(deltaHeading));
            deltaY = radius * Math.sin(deltaHeading);
        } else {
            // Robot moved in a straight line
            deltaX = deltaForwardInches;
            deltaY = 0;
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
        telemetry.addData("X Position", xPos);
        telemetry.addData("Y Position", yPos);
        telemetry.addData("Heading (rad)", heading);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.update();
    }

    private void driveToPosition(double targetX, double targetY, double targetHeading) {
        // Calculate errors
        double xError = targetX - xPos;
        double yError = targetY - yPos;
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

        // Set motor powers
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    private void motorStop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}