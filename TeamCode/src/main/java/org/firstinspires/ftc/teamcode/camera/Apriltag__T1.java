package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Autonomous", group = "Vision")
public class Apriltag__T1 extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Store detected ID
    int detectedTagID = -1;

    @Override
    public void runOpMode() {

        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        // Create vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();

        // Scan for tags BEFORE start
        while (!isStarted() && !isStopRequested()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.size() > 0) {
                detectedTagID = detections.get(0).id;
                telemetry.addData("Detected Tag ID", detectedTagID);
            } else {
                telemetry.addLine("No tag detected");
            }

            telemetry.update();
        }

        waitForStart();

        // Stop camera if you want (optional)
        // visionPortal.close();

        // Decide what to do based on tag ID
        if (detectedTagID == 1) {
            telemetry.addLine("Running LEFT autonomous");
            // driveLeft();
        }
        else if (detectedTagID == 2) {
            telemetry.addLine("Running CENTER autonomous");
            // driveCenter();
        }
        else if (detectedTagID == 3) {
            telemetry.addLine("Running RIGHT autonomous");
            // driveRight();
        }
        else {
            telemetry.addLine("No tag — running DEFAULT autonomous");
            // driveDefault();
        }

        telemetry.update();

        // Example delay so telemetry is visible
        sleep(3000);
    }
}
