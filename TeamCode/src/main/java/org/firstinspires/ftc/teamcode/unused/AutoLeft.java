package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoLeft", group = "MecanumBot")
@Disabled
public class AutoLeft extends LinearOpMode {

    public static final double motorPower = 0.5;
    public static final double armPower = 0.35;
    // Declare OpMode members.
    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;
    private DcMotor m4 = null;
    private DcMotor Arm;
    private Servo Claw;

    @Override
    public void runOpMode() {
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.REVERSE);
        Claw.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//         run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            motorRight();
            sleep(1550);
            motorStop();
            sleep(100);
            motorStraight();
            sleep(525);
            motorStop();
            sleep(100);
        }
        // Move the arm up
        Arm.setPower(armPower);
        sleep(3000);  // Move up for 1 second

        // Stop the arm
        Arm.setPower(0);
        sleep(500);// Stop for 0.5 second

        Claw.setPosition(0);
        sleep(1000);

        motorRight();
        sleep(100);
        motorStop();

        // Move the arm down
        Arm.setPower(-armPower);
        sleep(3000);  // Move down for 1 second

        // Stop the arm
        Arm.setPower(0);

        motorBack();
        sleep(450);
        motorStop();
        sleep(100);
        motorRight();
        sleep(2650);
        motorStop();
        sleep(100);

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