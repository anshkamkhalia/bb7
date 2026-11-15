package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class Teleop {
    private DcMotor m3; // front left
    private DcMotor m1; // front right
    private DcMotor m4; // back left
    private DcMotor m2; // back right

    private Servo side;
    private Servo pusher;

    private DcMotorEx outakeMotor;
    private DcMotor intakeMotor;

    // Servo limits (0.0 = full left, 1.0 = full right).
    private static final double SIDE_LEFT = 0.0;
    private static final double SIDE_RIGHT = 1.0;

    // Pusher positions. On a standard 180° servo, 0.0 = 0° and 1.0 = 180°.
    // (On a 270° or 300° servo, 180° would be 180/270 = 0.667 or 180/300 = 0.6.)
    private static final double PUSHER_REST = 0.0;   // 0°
    private static final double PUSHER_PUSH = 1.0;   // 180°

    // Widen the pusher's PWM range so 0.0–1.0 covers the servo's full travel.
    // The SDK default (600–2400 µs) often stops short of a full 180°.
    // If the servo buzzes/strains at either end, set this to false.
    private static final boolean PUSHER_FULL_RANGE = true;

    // How far a trigger must be pulled to count as "held"
    private static final double TRIGGER_THRESHOLD = 0.1;

    // Joystick deadzone so the outtake doesn't creep
    private static final double STICK_DEADZONE = 0.1;

    // TEMPORARY: PIDF velocity control turned off. Set to true to bring it back.
    private static final boolean USE_OUTTAKE_PIDF = true;

    // Plain-power outtake settings (used while USE_OUTTAKE_PIDF is false)
    private static final double OUTTAKE_POWER = 1.0;
    // Reverse is kept gentle so it backs a ball out instead of flinging it.
    private static final double OUTTAKE_REVERSE_POWER = -0.35;

    // Outtake PIDF (from Outtake PIDF Tuner: max vel 2200)
    private static final double OUTTAKE_P = 20.0;
    private static final double OUTTAKE_I = 0.0;
    private static final double OUTTAKE_D = 0.0;
    private static final double OUTTAKE_F = 17.617;

    // Outtake target velocity in ticks per second (PIDF mode only)
    private static final double OUTTAKE_TARGET_VELOCITY = 1800;

    // Outtake reverse velocity, ticks per second (PIDF mode only). Negative = backwards.
    private static final double OUTTAKE_REVERSE_VELOCITY = -800;

    // How close (ticks/sec) counts as "up to speed"
    private static final double OUTTAKE_TOLERANCE = 50;

    public void init(HardwareMap hwMap) {
        m3 = hwMap.get(DcMotor.class, "m3");
        m1 = hwMap.get(DcMotor.class, "m1");
        m4 = hwMap.get(DcMotor.class, "m4");
        m2 = hwMap.get(DcMotor.class, "m2");

        outakeMotor = hwMap.get(DcMotorEx.class, "outtake");
        intakeMotor = hwMap.get(DcMotor.class, "intake");

        side = hwMap.get(Servo.class, "side");
        side.setDirection(Servo.Direction.REVERSE); // servo direction reversed
        side.setPosition(SIDE_LEFT); // start as far left as possible

        pusher = hwMap.get(Servo.class, "pusher");
        pusher.setDirection(Servo.Direction.FORWARD); // pusher direction normal
        if (PUSHER_FULL_RANGE && pusher instanceof ServoImplEx) {
            ((ServoImplEx) pusher).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        pusher.setPosition(PUSHER_REST); // start at rest

        // Left side reversed
        m3.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        m1.setDirection(DcMotor.Direction.FORWARD);
        m2.setDirection(DcMotor.Direction.FORWARD);

        // Matches the tuner (outtake was flipped to FORWARD there)
        outakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Intake reversed
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (USE_OUTTAKE_PIDF) {
            // Encoder-based velocity control
            outakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(OUTTAKE_P, OUTTAKE_I, OUTTAKE_D, OUTTAKE_F));
        } else {
            // Plain power. The encoder still counts, so velocity telemetry still works.
            outakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        // Scale everything down if any value goes over 1
        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        m3.setPower(frontLeftPower / maxPower);
        m1.setPower(frontRightPower / maxPower);
        m4.setPower(backLeftPower / maxPower);
        m2.setPower(backRightPower / maxPower);
    }

    /**
     * Moves the side servo with gamepad2's right stick.
     * Released = full left, push right = moves right (full push = full right).
     */
    public void updateSide(Gamepad gamepad2) {
        double stickX = Range.clip(gamepad2.right_stick_x, 0.0, 1.0); // ignore left pushes
        side.setPosition(SIDE_LEFT + stickX * (SIDE_RIGHT - SIDE_LEFT));
    }

    /**
     * Pusher servo: hold gamepad2 A = swing to push position, release = back to rest.
     * Must be called every loop.
     */
    public void updatePusher(Gamepad gamepad2) {
        pusher.setPosition(gamepad2.a ? PUSHER_PUSH : PUSHER_REST);
    }

    /** Last commanded pusher position (0.0 to 1.0) for telemetry. */
    public double getPusherPosition() {
        return pusher.getPosition();
    }

    /**
     * Intake controls on gamepad2:
     * Left trigger = intake backwards (takes priority, e.g. to clear a jam).
     * Right trigger = intake forwards.
     * Neither = intake stops.
     */
    public void updateIntake(Gamepad gamepad2) {
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            reverseIntake();
        } else if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            startIntake();
        } else {
            stopIntake();
        }
    }

    /**
     * Outtake controls on gamepad2:
     * Left stick up OR left bumper = outtake spins backwards.
     * Left stick down = outtake spins forwards (1.0 power while PIDF is off).
     * Otherwise = outtake stops.
     */
    public void updateOuttake(Gamepad gamepad2) {
        double stickY = gamepad2.left_stick_y; // FTC: down is positive, up is negative

        if (gamepad2.left_bumper || stickY < -STICK_DEADZONE) {
            reverseOuttake();
        } else if (stickY > STICK_DEADZONE) {
            if (USE_OUTTAKE_PIDF) {
                // Harder push = faster, full push = target velocity
                outakeMotor.setVelocity(Range.clip(stickY, 0.0, 1.0) * OUTTAKE_TARGET_VELOCITY);
            } else {
                outakeMotor.setPower(OUTTAKE_POWER);
            }
        } else {
            endOuttake();
        }
    }

    public void startOuttake() {
        if (USE_OUTTAKE_PIDF) {
            outakeMotor.setVelocity(OUTTAKE_TARGET_VELOCITY);
        } else {
            outakeMotor.setPower(OUTTAKE_POWER);
        }
    }

    /** Runs the outtake backwards (e.g. to back out a stuck ball). */
    public void reverseOuttake() {
        if (USE_OUTTAKE_PIDF) {
            outakeMotor.setVelocity(OUTTAKE_REVERSE_VELOCITY);
        } else {
            outakeMotor.setPower(OUTTAKE_REVERSE_POWER);
        }
    }

    public void endOuttake() {
        if (USE_OUTTAKE_PIDF) {
            outakeMotor.setVelocity(0.0);
        } else {
            outakeMotor.setPower(0.0);
        }
    }

    /** Current outtake speed in ticks per second (for telemetry). */
    public double getOuttakeVelocity() {
        return outakeMotor.getVelocity();
    }

    /** True when the outtake is within tolerance of the target, i.e. ready to shoot. */
    public boolean outtakeAtSpeed() {
        return Math.abs(outakeMotor.getVelocity() - OUTTAKE_TARGET_VELOCITY) < OUTTAKE_TOLERANCE;
    }

    public void startIntake() {
        intakeMotor.setPower(1.0);
    }

    /** Runs the intake backwards (e.g. to spit out a jammed ball). */
    public void reverseIntake() {
        intakeMotor.setPower(-0.3);
    }

    public void stopIntake() {
        intakeMotor.setPower(0.0);
    }
}