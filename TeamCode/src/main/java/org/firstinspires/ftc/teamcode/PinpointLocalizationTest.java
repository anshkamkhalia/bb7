package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Localization Test", group = "Test")
public class PinpointLocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odom");

        // Same offsets as Constants.java
        pinpoint.setOffsets(7.875, -7.75, DistanceUnit.INCH);

        // CHANGE to goBILDA_SWINGARM_POD if you have swingarm pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Must match Constants.java
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,   // X (forward) pod
                GoBildaPinpointDriver.EncoderDirection.REVERSED); // Y (strafe) pod

        telemetry.addLine("Keep robot STILL - press START");
        telemetry.update();

        waitForStart();
        pinpoint.resetPosAndIMU(); // robot must be still for this

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            telemetry.addLine("Push FORWARD -> X goes UP");
            telemetry.addLine("Push LEFT -> Y goes UP");
            telemetry.addLine("Spin in place -> X and Y stay near 0");
            telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.update();
        }
    }
}