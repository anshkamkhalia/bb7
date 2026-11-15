package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagAutoAlignment extends OpMode {

    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // CAMERA PAN SERVO
    private Servo camServo;
    private double servoPos = 0.5;   // Start centered

    // If it still turns away, change this from 1 to -1
    private static final double DIRECTION = -1.0;

    // PD GAINS (from YouTuber style)
    double kP = 0.002;
    double kD = 0.0001;

    double error = 0;
    double lastError = 0;

    // We want bearing = 0 (tag centered)
    double goalBearing = 0.0;

    // Deadband to prevent jitter
    double angleTolerance = 0.4;

    double curTime = 0;
    double lastTime = 0;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        camServo = hardwareMap.get(Servo.class, "camServo");
        camServo.setPosition(servoPos);

        telemetry.addLine("Servo Tracker Initialized");
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
        lastTime = curTime;
    }

    @Override
    public void loop() {

        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        if (id20 != null) {

            // Bearing: how far off‑center the tag is
            // Positive = one side, negative = other side
            // error sign is then scaled by DIRECTION so you can flip it
            error = (id20.ftcPose.bearing - goalBearing) * DIRECTION;

            if (Math.abs(error) > angleTolerance) {

                curTime = getRuntime();
                double dt = curTime - lastTime;
                if (dt <= 0) dt = 0.01;

                double pTerm = error * kP;
                double dTerm = ((error - lastError) / dt) * kD;

                double output = pTerm + dTerm;

                // Apply PD output as a small increment to servo position
                servoPos += output;

                // Clamp servo range
                servoPos = Range.clip(servoPos, 0.0, 1.0);

                camServo.setPosition(servoPos);

                lastError = error;
                lastTime = curTime;
            }

            telemetry.addData("Tag 20 Bearing", id20.ftcPose.bearing);
            telemetry.addData("Error", error);
            telemetry.addData("Servo Pos", servoPos);

        } else {
            telemetry.addLine("Tag 20 not found");
            lastError = 0;
            lastTime = getRuntime();
        }
    }
}
