package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CheckMotor", group = "Linear Opmode")
@Disabled
public class CheckMotor extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private final int Outake_Slide_Position = 5000;
    // Define encoder positions for upper and lower limits
    private DcMotor Intake; //IntakeSlide renamed into Intake
    private DcMotor Outake2; //OutakeSlide renamed into Outake2
    private DcMotor Outake1;//OutakeRotation renamed into Outake1
    //private Servo IntakeRotation;// Delete this
    private Servo Pusher; // IntakeClaw renamed into Pusher
    private Servo Belt;//OutakeClaw1 renamed into Belt


    //private Servo OutakeClaw2; // Delete

    @Override
    public void runOpMode() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "m1"); //0
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor = hardwareMap.get(DcMotor.class, "m2"); // m2
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor = hardwareMap.get(DcMotor.class, "m3"); // m3
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor = hardwareMap.get(DcMotor.class, "m4"); // m4
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        //Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Intake.setTargetPosition(1);
        Belt = hardwareMap.get(Servo.class, "Belt");
        Outake1 = hardwareMap.get(DcMotor.class, "Outake1");
        //Outake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Outake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Outake1.setTargetPosition(1);
        Outake2 = hardwareMap.get(DcMotor.class, "Outake2");
        //Outake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outake2.setDirection(DcMotor.Direction.REVERSE);
        //Outake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Outake2.setTargetPosition(1);

        waitForStart();
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;  // Forward and backward movement
            double strafe = gamepad1.left_stick_x;  // Left and right movement (strafing)
            double rotate = gamepad1.right_stick_x; // Rotation (turning)

            // Calculate motor powers using mecanum formulas
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize motor powers so no value exceeds 1.0
            double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower /= 0.5;
                frontRightPower /= 0.5;
                backLeftPower /= 0.5;
                backRightPower /= 0.5;
            }
            frontLeftMotor.setPower(frontLeftPower/2);
            frontRightMotor.setPower(frontRightPower/2);
            backLeftMotor.setPower(backLeftPower/2);
            backRightMotor.setPower(backRightPower/2);

            if (gamepad1.right_trigger > 0 ) {
                frontLeftMotor.setPower(frontLeftPower*2);
                frontRightMotor.setPower(frontRightPower*2);
                backLeftMotor.setPower(backLeftPower*2);
                backRightMotor.setPower(backRightPower*2);
            }

            if (gamepad1.left_trigger>0) {
                frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
                backRightMotor.setDirection(DcMotor.Direction.REVERSE);
            } else {
                frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
                backRightMotor.setDirection(DcMotor.Direction.FORWARD);
            }
//            telemetry.addData("Front Left Power", frontLeftPower);
//            telemetry.addData("Front Right Power", frontRightPower);
//            telemetry.addData("Back Left Power", backLeftPower);
//            telemetry.addData("Back Right Power", backRightPower);
            if (gamepad1.a) {
                motorM1();
            } else if (gamepad1.b) {
                motorM2();
            } else if (gamepad1.x) {
                motorM3();
            } else if (gamepad1.y) {
                motorM4();
            }
            if (gamepad2.a) {
                Belt.setPosition(1);
            } else {
                Belt.setPosition(0);
            }

            if (gamepad2.left_trigger>0) {
                rotateIntakeUp();
            } else {
                Intake.setPower(0);
            }

            if (gamepad2.right_trigger>0) {
                Pusher.setPosition(0);
            } else {
                Pusher.setPosition(0.8);
            }

            if (gamepad2.left_stick_y>0) {
                //moveIntakeSlideToPosition(-Intake_Slide_Position, 1);  // Move up
                rotateOutTakeUp();
            } else if (gamepad2.left_stick_y==0){
                rotateOutTakeDown();
            }

        }
    }
    private void rotateOutTakeUp() {
            Outake1.setPower(1);
            Outake2.setPower(1);
    }
    private void rotateOutTakeDown() {
            Outake1.setPower(0);
            Outake2.setPower(0);
    }
    // Method to move the slide up
    private void rotateIntakeUp() {
        Intake.setPower(-1.0);// Move up with full power
    }

    // Method to move the slide down
    private void rotateIntakeDown() {
        if (Intake.getCurrentPosition() >= 0) {
            Intake.setPower(-1.0); // Move down with full power
            telemetry.addData(" IntakeSlide Down", Intake.getCurrentPosition());
            telemetry.update();
        }
        else if(Intake.getCurrentPosition() < 0) {
            Intake.setPower(0); // Stop if the lower limit is reached
            // RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //RightSlide.setTargetPosition(1);
            //RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

    }
    private void moveSlideUpOutTake() {
        if (Outake2.getCurrentPosition() <= Outake_Slide_Position) {
            Outake2.setPower(1);// Move up with full power
            telemetry.addData(" OutakeSlide UP", Outake2.getCurrentPosition());
            telemetry.update();
        }
        else if(Outake2.getCurrentPosition() > Outake_Slide_Position) {
            Outake2.setPower(0); // Stop if the upper limit is reached
            // RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // RightSlide.setTargetPosition(UPPER_LIMIT);
        }

    }

    // Method to move the slide down
    private void moveSlideDownOutTake() {
        if (Outake2.getCurrentPosition() >= 0) {
            Outake2.setPower(-1.0); // Move down with full power
            telemetry.addData(" OutakeSlide Down", Outake2.getCurrentPosition());
            telemetry.update();
        }
        else if(Outake2.getCurrentPosition() < 0) {
            Outake2.setPower(0); // Stop if the lower limit is reached
            // RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //RightSlide.setTargetPosition(1);
            //RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

    }


    private void moveIntakeSlideToPosition (int targetPosition, double power) {
        // Set the target position
//            RightSlide.setTargetPosition(targetPosition);
        Intake.setTargetPosition(targetPosition);
        // Set the motor to RUN_TO_POSITION mode
//            RightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set the motor power
//            RightSlide.setPower(power);
        Intake.setPower(power);
        // Wait until the motor reaches the target position
        while (Intake.isBusy()  && opModeIsActive()) {
            // Optionally, add telemetry or logic to stop if needed
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current IntakeSlide Position", Intake.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motor
        //RightSlide.setPower(0);
        Intake.setPower(0);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the motor back to RUN_USING_ENCODER mode for manual control
        // RightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void motorM1() {
        backRightMotor.setPower(1); //m1
        backLeftMotor.setPower(0); //m2
        frontRightMotor.setPower(0); //m3
        frontLeftMotor.setPower(0); //m4
        telemetry.addData("M1 backRightMotor", backRightMotor.getCurrentPosition() );
        telemetry.update();
    }
    private void motorM2() {
        backRightMotor.setPower(0); //m1
        backLeftMotor.setPower(1); //m2
        frontRightMotor.setPower(0); //m3
        frontLeftMotor.setPower(0); //m4
        telemetry.addData("M2 backLeftMotor", backLeftMotor.getCurrentPosition() );
        telemetry.update();
    }
    private void motorM3() {
        backRightMotor.setPower(0); //m1
        backLeftMotor.setPower(0); //m2
        frontRightMotor.setPower(1); //m3
        frontLeftMotor.setPower(0); //m4
        telemetry.addData("M3 frontRightMotor", frontRightMotor.getCurrentPosition() );
        telemetry.update();
    }
    private void motorM4() {
        backRightMotor.setPower(0); //m1
        backLeftMotor.setPower(0); //m2
        frontRightMotor.setPower(0); //m3
        frontLeftMotor.setPower(1); //m4
        telemetry.addData("M4 frontLeftMotor", frontLeftMotor.getCurrentPosition() );
        telemetry.update();
    }
    }



