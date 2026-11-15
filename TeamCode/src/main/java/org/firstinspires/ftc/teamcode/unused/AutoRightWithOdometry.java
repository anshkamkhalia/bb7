package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoRightWithOdometry", group = "MecanumBot")
@Disabled
public class AutoRightWithOdometry extends LinearOpMode {

    // Constants
    public static final double MOTOR_POWER = 0.5;
    public static final double TICKS_PER_REV = 8192; // Example value for an encoder
    public static final double WHEEL_DIAMETER = 1.0; // Diameter of odometry wheels in inches
    public static final double DISTANCE_PER_TICK = (Math.PI * WHEEL_DIAMETER) / TICKS_PER_REV;

    // Declare OpMode members
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;
    private DcMotor outakeSlide;
    private DcMotor outakeRotation;
    private Servo claw;
    private DcMotor yOdom;
    private DcMotor xOdom;

    // Odometry variables
    private int prevYTicks = 0;
    private int prevXTicks = 0;

    @Override
    public void runOpMode() {
        // Hardware initialization
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");
        claw = hardwareMap.get(Servo.class, "Claw");
        outakeSlide = hardwareMap.get(DcMotor.class, "OutakeSlide");
        outakeRotation = hardwareMap.get(DcMotor.class, "OutakeRotation");

        yOdom = hardwareMap.get(DcMotor.class, "YOdom");
        xOdom = hardwareMap.get(DcMotor.class, "XOdom");

        // Set motor directions
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);


        // Reset odometry encoders
        yOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //allow for motors to run without being impacted by encoder values
        yOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            moveToPosition(-28, 0); // Move 24 inches to the left
            motorStop();
            moveToPosition(-28, 36); // Move to (24, 36) coordinate
            motorStop();
        }
    }

    private void motorStop() {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    private void moveToPosition(double targetX, double targetY) {
        double currentX = 0, currentY = 0;
        while (opModeIsActive() & (Math.hypot(targetX - currentX, targetY - currentY) > 1.0)) {
            updateOdometry();

            // Placeholder for odometry logic to determine motor power adjustments
            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;

            double powerX = deltaX > 0 ? MOTOR_POWER : -MOTOR_POWER;
            double powerY = deltaY > 0 ? MOTOR_POWER : -MOTOR_POWER;

            m1.setPower(powerY);
            m2.setPower(powerY);
            m3.setPower(powerX);
            m4.setPower(powerX);

            telemetry.addData("Target X, Y", "%.2f, %.2f", targetX, targetY);
            telemetry.addData("Current X, Y", "%.2f, %.2f", currentX, currentY);
            telemetry.update();
        }
    }

    private void updateOdometry() {
        // Read encoder values
        int currentYTicks = yOdom.getCurrentPosition();
        int currentXTicks = xOdom.getCurrentPosition();

        // Calculate deltas
        int deltaY = currentYTicks - prevYTicks;
        int deltaX = currentXTicks - prevXTicks;

        // Update previous encoder values
        prevYTicks = currentYTicks;
        prevXTicks = currentXTicks;

        // Convert ticks to inches and update global coordinates (placeholders)
        double deltaYInches = deltaY * DISTANCE_PER_TICK;
        double deltaXInches = deltaX * DISTANCE_PER_TICK;
    }
}
