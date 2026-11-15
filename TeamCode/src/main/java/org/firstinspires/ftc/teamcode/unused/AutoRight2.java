package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto Code 2")
@Disabled
public class AutoRight2 extends LinearOpMode {
    // Hardware variables for motors and encoders
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftEncoder, rightEncoder, perpendicularEncoder;
    private DcMotor OutakeRotation;
    private Servo OutakeClaw1;
    private Servo OutakeClaw2;

    // Constants for odometry calculations
    private static final double COUNTS_PER_REV = 2000;  // Encoder ticks per revolution
    private static final double LEFT_RIGHT_WHEEL_DIAMETER = 1.25984;   // Diameter of odometry wheel (in inches)
    private static final double PERPENDICULAR_WHEEL_DIAMETER = 1.88976; // inches

    private static final double LEFT_RIGHT_WHEEL_CIRCUMFERENCE = Math.PI * LEFT_RIGHT_WHEEL_DIAMETER;
    private static final double PERPENDICULAR_WHEEL_CIRCUMFERENCE = Math.PI * PERPENDICULAR_WHEEL_DIAMETER;


    private static final double TRACK_WIDTH = 12.25;     // Distance between the two odometry wheels (in inches)
    private static final double PERPENDICULAR_OFFSET = 7.25; // Distance of perpendicular encoder from robot center (in inches)


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
            right();
            forward();

        }



    }
    private void right () {
        double targetX = 6; // Target X position in inches
        double targetY = 0; // Target Y position in inches
        double power = 0.5; // Target heading in radians
        // Drive to the target position
        driveToPosition(targetX, targetY, power);
        // Set motor power
        updateOdometry();
    }
    private void forward () {
        double targetX = 0; // Target X position in inches
        double targetY = 6; // Target Y position in inches
        double targetHeading = 0.5; // Target heading in radians
        // Drive to the target position
        driveToPosition(targetX, targetY, targetHeading);
        updateOdometry();
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
        double deltaLeftInches = (deltaLeft / COUNTS_PER_REV) * LEFT_RIGHT_WHEEL_CIRCUMFERENCE;
        double deltaRightInches = (deltaRight / COUNTS_PER_REV) * LEFT_RIGHT_WHEEL_CIRCUMFERENCE;
        double deltaPerpendicularInches =  (deltaPerpendicular / COUNTS_PER_REV) * PERPENDICULAR_WHEEL_CIRCUMFERENCE;

        // Calculate the change in heading

        double deltaHeading = (deltaRightInches - deltaLeftInches) / LEFT_RIGHT_WHEEL_DIAMETER;

        // Average forward movement
        double deltaForward = (deltaLeftInches + deltaRightInches) / 2.0;

        // Update global position
        xPos += deltaForward * Math.cos(heading) + deltaPerpendicularInches * Math.sin(heading);
        yPos += deltaForward * Math.sin(heading) - deltaPerpendicularInches * Math.cos(heading);
        heading += deltaHeading;

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

    private void driveToPosition(double targetX, double targetY, double power) {
        // Calculate errors
        final double tolerance = 0.5; // Tolerance in inches

        while (opModeIsActive() ) {
            updateOdometry();
            double xError = targetX - xPos;
            double yError = targetY - yPos;
            if (Math.hypot(xError, yError) < tolerance) {
                motorStop();
                break;
            }
            double targetHeading = Math.atan2(yError, xError);

            // Calculate power for each motor
            double drive = Math.cos(targetHeading - heading) * power;
            double strafe = Math.sin(targetHeading - heading) * power;
            double turn = 0; // Optional: Add proportional heading control if needed

            // Set motor powers
            double flPower = drive + strafe + turn;
            double frPower = drive - strafe - turn;
            double blPower = drive - strafe + turn;
            double brPower = drive + strafe - turn;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
        }

    }
    private void motorStop () {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        updateOdometry();

    }


    private void back () {
        double targetX = 0; // Target X position in inches
        double targetY = -6; // Target Y position in inches
        double targetHeading = 0.0; // Target heading in radians
        // Drive to the target position
        driveToPosition(targetX, targetY, targetHeading);
        updateOdometry();
    }
    private void sideabit () {
        double targetX = 6; // Target X position in inches
        double targetY = 0; // Target Y position in inches
        double targetHeading = 0.0; // Target heading in radians
        // Drive to the target position
        driveToPosition(targetX, targetY, targetHeading);
        updateOdometry();
    }

    // Check if the robot has reached the target position
    private boolean reachedTarget(double targetX, double targetY) {
        return (Math.abs(targetX - xPos) < 0.5) && (Math.abs(targetY - yPos) < 0.5);
    }
}

