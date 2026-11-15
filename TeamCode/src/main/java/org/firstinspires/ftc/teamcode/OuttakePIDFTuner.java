package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/*
 * OUTTAKE PIDF TUNER (gamepad1)
 *
 *   A                 = start / stop outtake at target velocity
 *   X                 = max velocity test (full power, no PID) - press again to exit
 *   Left/Right bumper = select which value to edit (P, I, D, F, Target)
 *   Dpad left/right   = change step size
 *   Dpad up/down      = increase / decrease selected value
 *
 *   B (hold)          = pusher to push position, release = back to rest
 *   Right trigger     = intake forwards
 *   Left trigger      = intake backwards (takes priority)
 *   Y                 = reset the "min vel" shot-dip readout
 */
@TeleOp(name = "Outtake PIDF Tuner")
public class OuttakePIDFTuner extends OpMode {

    private DcMotorEx outtake;
    private DcMotor intake;
    private Servo pusher;

    // Pusher settings - match Teleop
    private static final double PUSHER_REST = 0.0;
    private static final double PUSHER_PUSH = 1.0;
    private static final boolean PUSHER_FULL_RANGE = true;

    private static final double TRIGGER_THRESHOLD = 0.1;

    private double p, i, d, f;
    private double targetVelocity = 1500; // ticks per second

    private static final String[] NAMES = {"P", "I", "D", "F", "Target"};
    private int selected = 0;

    private static final double[] STEPS = {0.001, 0.01, 0.1, 1.0, 10.0, 100.0};
    private int stepIndex = 3; // start at 1.0

    private boolean running = false;
    private boolean maxTest = false;
    private double maxVelocitySeen = 0;

    // Lowest velocity seen while running (shows how much a shot drags the flywheel down)
    private double minVelocitySeen = Double.MAX_VALUE;

    // Previous button states for edge detection (one press = one change)
    private boolean prevUp, prevDown, prevLeft, prevRight, prevLB, prevRB, prevA, prevX, prevY;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorSimple.Direction.FORWARD); // match your Teleop class
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE); // match Teleop
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setDirection(Servo.Direction.REVERSE); // match Teleop
        if (PUSHER_FULL_RANGE && pusher instanceof ServoImplEx) {
            ((ServoImplEx) pusher).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        pusher.setPosition(PUSHER_REST);

        // Start from whatever the hub currently uses, with I and D zeroed
        PIDFCoefficients current = outtake.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        p = current.p;
        i = 0.0;
        d = 0.0;
        f = current.f;
        applyPIDF();

        telemetry.addLine("Outtake PIDF Tuner ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean up    = gamepad1.dpad_up      && !prevUp;
        boolean down  = gamepad1.dpad_down    && !prevDown;
        boolean left  = gamepad1.dpad_left    && !prevLeft;
        boolean right = gamepad1.dpad_right   && !prevRight;
        boolean lb    = gamepad1.left_bumper  && !prevLB;
        boolean rb    = gamepad1.right_bumper && !prevRB;
        boolean a     = gamepad1.a            && !prevA;
        boolean x     = gamepad1.x            && !prevX;
        boolean y     = gamepad1.y            && !prevY;

        // Select value to edit
        if (rb) selected = (selected + 1) % NAMES.length;
        if (lb) selected = (selected - 1 + NAMES.length) % NAMES.length;

        // Change step size
        if (right) stepIndex = Math.min(stepIndex + 1, STEPS.length - 1);
        if (left)  stepIndex = Math.max(stepIndex - 1, 0);

        // Change selected value
        if (up)   adjustSelected(STEPS[stepIndex]);
        if (down) adjustSelected(-STEPS[stepIndex]);

        // Toggle running at target velocity
        if (a && !maxTest) {
            running = !running;
            minVelocitySeen = Double.MAX_VALUE;
        }

        // Reset shot-dip readout
        if (y) minVelocitySeen = Double.MAX_VALUE;

        // Toggle max velocity test
        if (x) {
            maxTest = !maxTest;
            if (maxTest) {
                running = false;
                maxVelocitySeen = 0;
                outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                outtake.setPower(0);
                outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                applyPIDF();
            }
        }

        double velocity = outtake.getVelocity();

        if (maxTest) {
            outtake.setPower(1.0);
            maxVelocitySeen = Math.max(maxVelocitySeen, Math.abs(velocity));
        } else if (running) {
            outtake.setVelocity(targetVelocity);
            minVelocitySeen = Math.min(minVelocitySeen, velocity);
        } else {
            outtake.setPower(0);
        }

        // Pusher: hold B to push
        pusher.setPosition(gamepad1.b ? PUSHER_PUSH : PUSHER_REST);

        // Intake: left trigger = reverse (priority), right trigger = forward
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            intake.setPower(-1.0);
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }

        // Telemetry
        telemetry.addData("Mode", maxTest ? "MAX VELOCITY TEST" : (running ? "RUNNING" : "STOPPED"));
        telemetry.addLine();
        for (int k = 0; k < NAMES.length; k++) {
            String marker = (k == selected) ? ">> " : "   ";
            telemetry.addData(marker + NAMES[k], "%.4f", getValue(k));
        }
        telemetry.addData("Step", STEPS[stepIndex]);
        telemetry.addLine();
        telemetry.addData("Target vel", "%.0f", running ? targetVelocity : 0.0);
        telemetry.addData("Actual vel", "%.0f", velocity);
        telemetry.addData("Error", "%.0f", running ? targetVelocity - velocity : 0.0);
        telemetry.addData("Motor power", "%.2f", outtake.getPower());
        if (running && minVelocitySeen != Double.MAX_VALUE) {
            telemetry.addData("Min vel (Y resets)", "%.0f", minVelocitySeen);
            telemetry.addData("Shot dip", "%.0f", targetVelocity - minVelocitySeen);
        }

        telemetry.addLine();
        telemetry.addData("Pusher", gamepad1.b ? "PUSH" : "REST");
        telemetry.addData("Intake power", "%.1f", intake.getPower());

        if (maxTest || maxVelocitySeen > 0) {
            telemetry.addLine();
            telemetry.addData("Max vel seen", "%.0f", maxVelocitySeen);
            if (maxVelocitySeen > 0) {
                telemetry.addData("Suggested F", "%.3f", 32767.0 / maxVelocitySeen);
            }
        }
        telemetry.update();

        prevUp = gamepad1.dpad_up;       prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;   prevRight = gamepad1.dpad_right;
        prevLB = gamepad1.left_bumper;   prevRB = gamepad1.right_bumper;
        prevA = gamepad1.a;              prevX = gamepad1.x;
        prevY = gamepad1.y;
    }

    @Override
    public void stop() {
        outtake.setPower(0);
        intake.setPower(0);
        pusher.setPosition(PUSHER_REST);
    }

    private void adjustSelected(double delta) {
        switch (selected) {
            case 0: p = Math.max(0, p + delta); break;
            case 1: i = Math.max(0, i + delta); break;
            case 2: d = Math.max(0, d + delta); break;
            case 3: f = Math.max(0, f + delta); break;
            case 4: targetVelocity = Math.max(0, targetVelocity + delta); break;
        }
        if (selected <= 3) applyPIDF();
    }

    private double getValue(int index) {
        switch (index) {
            case 0: return p;
            case 1: return i;
            case 2: return d;
            case 3: return f;
            default: return targetVelocity;
        }
    }

    private void applyPIDF() {
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(p, i, d, f));
    }
}