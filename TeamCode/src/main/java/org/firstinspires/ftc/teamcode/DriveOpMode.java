package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive OpMode")
public class DriveOpMode extends OpMode {

    private Teleop drivetrain = new Teleop();

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Stick Y is negative when pushed forward, so flip it
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Deadzone so brake mode works when sticks are released
        if (Math.abs(forward) < 0.05) forward = 0;
        if (Math.abs(strafe) < 0.05) strafe = 0;
        if (Math.abs(rotate) < 0.05) rotate = 0;

        drivetrain.drive(forward, strafe, rotate);

        // Outtake: gamepad2 left stick down = spin up, hold B = reverse (B wins)
        drivetrain.updateOuttake(gamepad2);

        // Intake: right trigger = forward, left trigger = backward
        drivetrain.updateIntake(gamepad2);

        // Side servo rests full left; push gamepad2 right stick right to move it right
        drivetrain.updateSide(gamepad2);

        // Pusher: hold gamepad2 A to swing to 180°, release to return to 0°
        drivetrain.updatePusher(gamepad2);

        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Rotate", rotate);
        telemetry.addData("Outtake Stick Y", gamepad2.left_stick_y);
        telemetry.addData("Outtake Reverse (B)", gamepad2.b);
        telemetry.addData("Outtake Velocity", "%.0f", drivetrain.getOuttakeVelocity());
        telemetry.addData("Outtake At Speed", drivetrain.outtakeAtSpeed());
        telemetry.addData("Intake Fwd (RT)", gamepad2.right_trigger);
        telemetry.addData("Intake Rev (LT)", gamepad2.left_trigger);
        telemetry.addData("Side Stick", gamepad2.right_stick_x);
        telemetry.addData("Pusher A", gamepad2.a);
        telemetry.addData("Pusher Pos", drivetrain.getPusherPosition());
        telemetry.update();
    }
}