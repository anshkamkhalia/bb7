package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Check", group = "Test")
public class OdomTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odom");

        odo.setOffsets(-1.0954051130399929, -0.8266564617006797, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();   // keep the robot still for a moment after init

        while (opModeInInit()) {
            odo.update();
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addLine("Keep robot still, then press Start");
            telemetry.update();
        }

        while (opModeIsActive()) {
            odo.update();
            if (gamepad1.a) odo.resetPosAndIMU();

            Pose2D pos = odo.getPosition();
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Raw X encoder", odo.getEncoderX());
            telemetry.addData("Raw Y encoder", odo.getEncoderY());
            telemetry.addData("X (in)", "%.2f", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint Hz", "%.0f", odo.getFrequency());
            telemetry.addLine("Press A to reset");
            telemetry.update();
        }
    }
}