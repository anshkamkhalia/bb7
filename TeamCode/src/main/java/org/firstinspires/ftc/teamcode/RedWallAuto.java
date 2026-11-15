package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.unused.AprilTagWebcam;

@Autonomous(name="RedWall Auto Simplified", group = "Mr. Phil")
public class RedWallAuto extends LinearOpMode {

    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor Intake;
    private DcMotor Outake2;
    private Servo Pusher;
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // ===== HOLD TIME VARIABLES =====
    private static final double HOLD_FAST = 0.02;
    private static final double HOLD_SMALL = 0.04;
    private static final double HOLD_ALIGN = 0.06;

    @Override
    public void runOpMode() {

        robot.initialize(true);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Outake2 = hardwareMap.get(DcMotor.class, "Outake2");
        Outake2.setDirection(DcMotor.Direction.REVERSE);

        Pusher = hardwareMap.get(Servo.class, "Pusher");

        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            Pusher.setPosition(0.5);

            // ===== FIRST SHOOTING CYCLE =====
            Outake2.setPower(0.885);
            robot.drive(-55, 1, 0.01);

            Intake.setPower(0.75);
            sleep(200);

            Outake2.setPower(0.96);
            Intake.setPower(-0.75);
            sleep(200);

            Intake.setPower(0.75);
            sleep(1000);

            Pusher.setPosition(0);
            sleep(100);
            Pusher.setPosition(0.5);

            Intake.setPower(0);
            Outake2.setPower(0);

            // ===== SECOND CYCLE =====
            robot.turnTo(-53, 0.65, HOLD_ALIGN); // flipped

            Intake.setPower(1);
            Outake2.setPower(-0.8);

            robot.drive(33, 0.9, HOLD_FAST);
            robot.drive(-31, 1, HOLD_SMALL);

            Intake.setPower(-0.75);
            sleep(150);
            Intake.setPower(0);

            Outake2.setPower(-0.8);
            sleep(150);

            Outake2.setPower(0.92);

            robot.turnTo(2, 0.5, HOLD_ALIGN); // flipped
            sleep(500);

            Intake.setPower(0.75);
            sleep(300);
            Intake.setPower(-0.75);
            sleep(200);
            Outake2.setPower(0.95);
            Intake.setPower(0.75);
            sleep(1500);

            Pusher.setPosition(0);
            sleep(600);
            Pusher.setPosition(0.5);

            Intake.setPower(0);
            Outake2.setPower(0);

            // ===== THIRD CYCLE =====
            robot.drive(-8, 0.9, HOLD_FAST);
            robot.strafe(-35, 1, HOLD_SMALL); // flipped

            Intake.setPower(0.75);

            robot.turnTo(-40, 0.5, HOLD_ALIGN); // flipped
            robot.drive(17.4, 1, 0.01);

            Intake.setPower(-0.75);
            sleep(150);
            Intake.setPower(0);

            Outake2.setPower(-0.80);
            sleep(80);
            Outake2.setPower(0);

            robot.drive(-30, 0.96, HOLD_SMALL);
            robot.strafe(30, 0.6, HOLD_SMALL); // flipped

            Outake2.setPower(0.926);

            robot.turnTo(4, 0.6, HOLD_ALIGN); // flipped
            sleep(500);

            Intake.setPower(0.75);
            sleep(240);
            Intake.setPower(-0.75);
            sleep(200);
            Intake.setPower(0.75);
            sleep(1000);

            Pusher.setPosition(0);
            sleep(200);
            Pusher.setPosition(0.5);

            robot.resetZero();
        }
    }
}
