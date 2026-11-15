package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoRightWithOdometry2", group = "MecanumBot")
@Disabled
public class AutoRightWithOdometry2 extends LinearOpMode {

    // Constants
    // public static final double motorPower = 0.5;
   // public static final double armPower = 0.35;
    public static final double TICKS_PER_REV = 537.6; // Example value for an encoder
    public static final double WHEEL_DIAMETER = 1.26; // Diameter of odometry wheels in inches
    public static final double DISTANCE_PER_TICK = (Math.PI * WHEEL_DIAMETER) / TICKS_PER_REV;
    public static final double TRACK_WIDTH = 3.5; // Distance between y and right odometry wheels in inches

    // Declare OpMode members
    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;
    private DcMotor m4 = null;
    //private DcMotor Arm = null;
    //private Servo Claw = null;
    private DcMotor OutakeSlide = null; // Added OutakeSlide motor
    private DcMotor OutakeRotation = null; // Added OutakeRotation motor

    // OdometrY wheels
    private DcMotor YOdom = null;
    private DcMotor XOdom = null;

    // Global robot state
    private double globalX = 0.0;
    private double globalY = 0.0;
    private double globalTheta = 0.0;

    private int prevYTicks = 0;
    private int prevXTicks = 0;

    @Override
    public void runOpMode() {
        // Hardware initialization
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");
        //Arm = hardwareMap.get(DcMotor.class, "Arm");
        //Claw = hardwareMap.get(Servo.class, "Claw");
        OutakeSlide = hardwareMap.get(DcMotor.class, "OutakeSlide"); // Initialize OutakeSlide
        OutakeRotation = hardwareMap.get(DcMotor.class, "OutakeRotation"); // Initialize OutakeRotation

        YOdom = hardwareMap.get(DcMotor.class, "YOdom");
        XOdom = hardwareMap.get(DcMotor.class, "XOdom");

        // Set motor directions
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        //Arm.setDirection(DcMotor.Direction.REVERSE);
        OutakeSlide.setDirection(DcMotor.Direction.FORWARD); // Set OutakeSlide direction (if needed)
        OutakeRotation.setDirection(DcMotor.Direction.FORWARD); // Set OutakeRotation direction (if needed)

        // Reset odometry encoders
        YOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        XOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        YOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        XOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Autonomous routine
        if (opModeIsActive()) {
            moveToPosition(24, 0); // Move 24 inches forward
            motorStop();
            moveToPosition(24, -18); // Move to (24, 24) coordinate
            motorStop();
        }
    }

    private void updateOdometry() {
        // Read encoder values
        int currentYTicks = YOdom.getCurrentPosition();
        int currentXTicks = XOdom.getCurrentPosition();

        // Calculate deltas
        int deltaY = currentYTicks - prevYTicks;
        int deltaX = currentXTicks - prevXTicks;

        // Update previous encoder values
        prevYTicks = currentYTicks;
        prevXTicks = currentXTicks;

        // Convert encoder ticks to distances
        double deltaYDist = deltaY * DISTANCE_PER_TICK;
        double deltaXDist = deltaX * DISTANCE_PER_TICK;

        // Calculate robot rotation (theta)
        double deltaTheta = (deltaXDist - deltaYDist) / TRACK_WIDTH;

        // Update global theta and normalize
        globalTheta += deltaTheta;
        globalTheta = (globalTheta + Math.PI) % (2 * Math.PI) - Math.PI;

        // Calculate field-centric position changes
        double deltaLateralDist = (deltaXDist + deltaYDist) / 2.0; // Added missing calculation
        double deltaXField = deltaLateralDist * Math.cos(globalTheta) - deltaYDist * Math.sin(globalTheta);
        double deltaYField = deltaLateralDist * Math.sin(globalTheta) + deltaYDist * Math.cos(globalTheta);

        // Update global position
        globalX += deltaXField;
        globalY += deltaYField;
    }

    private void moveToPosition(double targetX, double targetY) {
        double errorX, errorY;

        do {
            updateOdometry();

            // Calculate position errors
            errorX = targetX - globalX;
            errorY = targetY - globalY;

            // Calculate motor powers (simplified P-control)
            double powerX = errorX * 0.1; // Adjust P-gain as needed
            double powerY = errorY * 0.1;

            // Clamp motor powers
            double powerLimit = 1.0;
            powerX = Math.max(-powerLimit, Math.min(powerLimit, powerX)); // Fixed Math.maX to Math.max
            powerY = Math.max(-powerLimit, Math.min(powerLimit, powerY)); // Fixed Math.maX to Math.max

            // Apply power to motors
            m1.setPower(powerY + powerX);
            m2.setPower(powerY - powerX);
            m3.setPower(powerY + powerX);
            m4.setPower(powerY - powerX);

        } while (opModeIsActive() && (Math.abs(errorX) > 0.5 || Math.abs(errorY) > 0.5));
    }

    private void motorStop() {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        OutakeSlide.setPower(0); // Stop OutakeSlide motor
        OutakeRotation.setPower(0); // Stop OutakeRotation motor
    }
}
