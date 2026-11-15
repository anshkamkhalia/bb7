package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Drive2", group = "Linear Opmode")
@Disabled
public class Teleop1 extends LinearOpMode {

    // Declare motor variables
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor LeftSlide;
    private DcMotor RightSlide;
    private final int SLIDE_TARGET_POSITION = 3500;
    // Define encoder positions for upper and lower limits
    private static final int LOWER_LIMIT = -100; // Encoder position for fully down
    private static final int UPPER_LIMIT = -4300; // Encoder position for fully up (tune this value based on your robot)
    private DcMotor Arm;



    @Override
    public void runOpMode() {
        RightSlide = hardwareMap.get(DcMotor.class, "RightSlide");
        LeftSlide = hardwareMap.get(DcMotor.class, "LeftSlide");
        RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftSlide.setDirection(DcMotor.Direction.REVERSE);


        RightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {



            double slidePower = -gamepad2.right_stick_y;  // Forward and backward movement


            moveSlide(slidePower);
            // Telemetry to see the position and motor power
            telemetry.addData("Right Slide Position 2", RightSlide.getCurrentPosition());
            telemetry.addData("Left Slide Position 2", LeftSlide.getCurrentPosition());

            telemetry.addData("Motor Power 2", RightSlide.getPower());
            telemetry.update();
        }
    }
    //Method to move the slide up
    private void moveSlide(double power) {
        if (RightSlide.getCurrentPosition() > UPPER_LIMIT) {
            RightSlide.setPower(power); // Move up with full power
        } else {
            RightSlide.setPower(0); // Stop if the upper limit is reached
        }
        if (LeftSlide.getCurrentPosition() > UPPER_LIMIT) {
            LeftSlide.setPower(power); // Move up with full power
        } else {
            LeftSlide.setPower(0); // Stop if the upper limit is reached
        }
        // Method to move the slide down
        if (RightSlide.getCurrentPosition() < LOWER_LIMIT) {
            RightSlide.setPower(-power); // Move down with full power
        } else {
            RightSlide.setPower(0); // Stop if the lower limit is reached
        }
        if (LeftSlide.getCurrentPosition() < LOWER_LIMIT) {
            LeftSlide.setPower(-power ); // Move down with full power
        } else {
            LeftSlide.setPower(0); // Stop if the lower limit is reached
        }
        RightSlide.setPower(power); // Move up with full power
        LeftSlide.setPower(power); // Move up with full power
    }

    //Method to move the slide down

}