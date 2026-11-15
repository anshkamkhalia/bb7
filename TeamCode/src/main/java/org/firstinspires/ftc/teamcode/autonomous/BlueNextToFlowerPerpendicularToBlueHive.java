package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "BlueNextToFlowerPerpendicularToBlueHive", group = "Blue")
public class BlueNextToFlowerPerpendicularToBlueHive extends OpMode {

    private Follower follower;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private int state = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Poses.BLUE_FLOWER_PERP_HIVE_ROBOT_START);
        // Initialize subsystems here (intake, arm, claw, etc.)
    }

    @Override
    public void init_loop() {
        telemetry.addLine("BlueNextToFlowerPerpendicularToBlueHive ready");
        telemetry.addData("Place robot at", "(%.1f, %.1f) %.0f°",
                Poses.BLUE_FLOWER_PERP_HIVE_ROBOT_START.x(),
                Poses.BLUE_FLOWER_PERP_HIVE_ROBOT_START.y(),
                Math.toDegrees(Poses.BLUE_FLOWER_PERP_HIVE_ROBOT_START.heading()));
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
        telemetry.addData("X", "%.2f", pose.x());
        telemetry.addData("Y", "%.2f", pose.y());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    private void updateStateMachine() {
        switch (state) {
            case 0: // Drive the path
                follower.follow(AutoPaths.blueNextToFlowerPerpendicularToBlueHive());
                setState(1);
                break;

            case 1: // Wait for the path to finish
                if (!follower.isBusy()) {
                    // Start an action here, e.g. score or intake
                    setState(2);
                }
                break;

            case 2: // Wait for the action to finish (placeholder: 0.5 s)
                if (stateTimer.seconds() > 0.5) {
                    setState(-1);
                }
                break;

            default: // Done: follower holds its final position
                break;
        }
    }

    private void setState(int newState) {
        state = newState;
        stateTimer.reset();
    }
}