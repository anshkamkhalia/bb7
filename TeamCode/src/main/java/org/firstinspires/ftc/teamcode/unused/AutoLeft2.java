package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoLeft2", group = "MecanumBot")
@Disabled
public class AutoLeft2 extends LinearOpMode {
    public static final double motorPower = 0.5;
    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;
    private DcMotor m4 = null;
    private DcMotor OutakeRotation;
    private DcMotor OutakeSlide;
    private Servo OutakeClaw1;
    private Servo OutakeClaw2;
    private final int Outake_Slide_Position = 1500;
    private final int Outake_Rotate_Position = 9000;



    @Override
    public void runOpMode() {
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");
        //Arm = hardwareMap.get(DcMotor.class, "Arm");
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        //Arm.setDirection(DcMotor.Direction.REVERSE);
        OutakeClaw1 = hardwareMap.get(Servo.class, "OutakeClaw1");
        OutakeClaw2 = hardwareMap.get(Servo.class, "OutakeClaw2");
        OutakeClaw2.setDirection(Servo.Direction.REVERSE);
        OutakeRotation = hardwareMap.get(DcMotor.class, "OutakeRotation");
        OutakeRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OutakeRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OutakeRotation.setTargetPosition(0);
        OutakeSlide = hardwareMap.get(DcMotor.class, "OutakeSlide");
        OutakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OutakeSlide.setDirection(DcMotor.Direction.REVERSE);
        OutakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OutakeSlide.setTargetPosition(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            OutakeClaw1.setPosition(1);
            OutakeClaw2.setPosition(1);
            sleep(3000);
            motorRight();
            sleep(4000);
            motorStop();
            sleep(1000);
        }



    }
    private void moveMotorToPosition(int targetPosition, double power, DcMotor test1) {
        // Set the target position
        test1.setTargetPosition(targetPosition);

        // Set the mode to RUN_TO_POSITION
        test1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor power
        test1.setPower(power);

        // Wait for the motor to reach the target position
        while (opModeIsActive() && test1.isBusy()) {
            telemetry.addData("Motor Target", targetPosition);
            telemetry.addData("Motor Current", test1.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motor
        test1.setPower(0);

        // Set the motor mode back to RUN_USING_ENCODER
        test1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void motorBack() {
        m1.setPower(-motorPower);
        m2.setPower(-motorPower);
        m3.setPower(-motorPower);
        m4.setPower(-motorPower);
    }

    private void motorRight() {
        m1.setPower(motorPower);
        m2.setPower(-motorPower);
        m3.setPower(-motorPower);
        m4.setPower(motorPower);
    }

    private void motorStraight() {
        m1.setPower(motorPower);
        m2.setPower(motorPower);
        m3.setPower(motorPower);
        m4.setPower(motorPower);
    }

    private void motorStop() {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}