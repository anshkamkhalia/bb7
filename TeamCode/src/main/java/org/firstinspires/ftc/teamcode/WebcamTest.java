package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.unused.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class WebcamTest extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // Update the vision portal
        aprilTagWebcam.update();
        
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        
        if (id20 != null) {
            telemetry.addData("Target Found", "ID %d", id20.id);
            telemetry.addData("Translation", "X: %.2f, Y: %.2f", id20.ftcPose.x, id20.ftcPose.y);
            telemetry.addData("Rotation", "Yaw: %.2f", id20.ftcPose.yaw);
        } else {
            telemetry.addLine("Searching for Tag 20...");
        }

        // Show total count of tags seen (useful for debugging)
        telemetry.addData("Total Tags Seen", aprilTagWebcam.getDetectedTags().size());
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
