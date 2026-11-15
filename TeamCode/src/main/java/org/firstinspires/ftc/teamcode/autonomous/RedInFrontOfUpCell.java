package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "RedInFrontOfUpCell", group = "Red")
public class RedInFrontOfUpCell extends OpMode {

    private Follower follower;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int state = 0;

    private DcMotorEx outtakeMotor;
    private DcMotor intakeMotor;

    // Same PIDF as Teleop (from Outtake PIDF Tuner)
    private static final double OUTTAKE_P = 20.0;
    private static final double OUTTAKE_I = 0.0;
    private static final double OUTTAKE_D = 0.0;
    private static final double OUTTAKE_F = 17.617;

    private static final double OUTTAKE_TARGET_VELOCITY = 1800; // ticks/sec
    private static final double OUTTAKE_TOLERANCE = 50;         // ticks/sec

    // Timing
    private static final double FEED_TIME = 6.3; // run intake this long after outtake reaches speed

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.RED_START);

        // Outtake: velocity control with PIDF
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(OUTTAKE_P, OUTTAKE_I, OUTTAKE_D, OUTTAKE_F));
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake: drives all transfers, feeds balls into the outtake
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("RedInFrontOfUpCell ready");
        telemetry.addData("Place robot at", "(%.1f, %.1f) %.0f°",
                Poses.RED_START.x(), Poses.RED_START.y(),
                Math.toDegrees(Poses.RED_START.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        setState(0);
    }

    @Override
    public void loop() {
        follower.update();      // must run every loop
        updateStateMachine();

        Pose pose = follower.pose();
        telemetry.addData("State", state);
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Outtake vel", "%.0f / %.0f",
                outtakeMotor.getVelocity(), OUTTAKE_TARGET_VELOCITY);
        telemetry.addData("X", "%.2f", pose.x());
        telemetry.addData("Y", "%.2f", pose.y());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        outtakeMotor.setVelocity(0.0);
        intakeMotor.setPower(0.0);
    }

    private void updateStateMachine() {
        switch (state) {
            case 0: // Spin up outtake
                outtakeMotor.setVelocity(OUTTAKE_TARGET_VELOCITY);
                setState(1);
                break;

            case 1: // Wait until outtake reaches target velocity
                if (outtakeAtSpeed()) {
                    intakeMotor.setPower(1.0); // feed balls into the shooter
                    setState(2);
                }
                break;

            case 2: // Shooting: keep feeding for FEED_TIME
                if (stateTimer.seconds() > FEED_TIME) {
                    intakeMotor.setPower(0.0);
                    outtakeMotor.setVelocity(0.0);
                    follower.follow(AutoPaths.redPath1()); // drive to park
                    setState(3);
                }
                break;

            case 3: // Wait for park path to finish
                if (!follower.isBusy()) {
                    setState(-1);
                }
                break;

            default: // Done: follower holds its final position
                break;
        }
    }

    private boolean outtakeAtSpeed() {
        return Math.abs(outtakeMotor.getVelocity() - OUTTAKE_TARGET_VELOCITY) < OUTTAKE_TOLERANCE;
    }

    private void setState(int newState) {
        state = newState;
        stateTimer.reset();
    }
}
//
//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.pedro.Constants;
//
//@Autonomous(name = "RedInFrontOfUpCell", group = "Red")
//public class RedInFrontOfUpCell extends OpMode {
//
//    private Follower follower;
//    private final ElapsedTime stateTimer = new ElapsedTime();
//    private int state = 0;
//
//    private DcMotorEx outtakeMotor;
//    private DcMotor intakeMotor;
//
//    // Same PIDF as Teleop (from Outtake PIDF Tuner). Not used while auto is park-only.
//    private static final double OUTTAKE_P = 20.0;
//    private static final double OUTTAKE_I = 0.0;
//    private static final double OUTTAKE_D = 0.0;
//    private static final double OUTTAKE_F = 17.617;
//
//    @Override
//    public void init() {
//        follower = Constants.create(hardwareMap);
//        follower.setPose(Poses.RED_START);
//
//        // Outtake: set up but not run (park-only for now)
//        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
//        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(OUTTAKE_P, OUTTAKE_I, OUTTAKE_D, OUTTAKE_F));
//        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        // Intake: set up but not run (park-only for now)
//        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//    }
//
//    @Override
//    public void init_loop() {
//        telemetry.addLine("RedInFrontOfUpCell ready (park only)");
//        telemetry.addData("Place robot at", "(%.1f, %.1f) %.0f°",
//                Poses.RED_START.x(), Poses.RED_START.y(),
//                Math.toDegrees(Poses.RED_START.heading()));
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        setState(0);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();      // must run every loop
//        updateStateMachine();
//
//        Pose pose = follower.pose();
//        telemetry.addData("State", state);
//        telemetry.addData("Mode", follower.mode());
//        telemetry.addData("X", "%.2f", pose.x());
//        telemetry.addData("Y", "%.2f", pose.y());
//        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading()));
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        outtakeMotor.setVelocity(0.0);
//        intakeMotor.setPower(0.0);
//    }
//
//    private void updateStateMachine() {
//        switch (state) {
//            case 0: // Start the park path
//                follower.follow(AutoPaths.redPath1());
//                setState(1);
//                break;
//
//            case 1: // Wait for park path to finish
//                if (!follower.isBusy()) {
//                    setState(-1);
//                }
//                break;
//
//            default: // Done: follower holds its final position
//                break;
//        }
//    }
//
//    private void setState(int newState) {
//        state = newState;
//        stateTimer.reset();
//    }
//}