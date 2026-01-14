package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@TeleOp(name="Camera_3", group="Paste")
public class Test3 extends LinearOpMode {

    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId
        );

        webcam.setPipeline(new SpherePipeline());
        webcam.openCameraDevice();
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        telemetry.addData("Status", "Camera initializata");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Striming", "Camera activata");
            telemetry.update();
            sleep(100);
        }
    }

    // ============================================================
    // ======================= PIPELINE ============================
    // ============================================================

    class SpherePipeline extends OpenCvPipeline {

        // ------- PRE-CREATE MATS (NO MEMORY LEAKS) -------
        private Mat hsv = new Mat();
        private Mat gray = new Mat();
        private Mat circles = new Mat();

        private Mat roiHSV = new Mat();
        private Mat maskGreen = new Mat();
        private Mat maskPurple1 = new Mat();
        private Mat maskPurple2 = new Mat();
        private Mat maskPurple = new Mat();

        // ------- COLOR RANGES -------
        private Scalar lowerGreen = new Scalar(50, 100, 100);
        private Scalar upperGreen = new Scalar(70, 255, 255);

        private Scalar lowerPurple1 = new Scalar(129, 80, 80);
        private Scalar upperPurple1 = new Scalar(158, 255, 255);

        private Scalar lowerPurple2 = new Scalar(158, 80, 80);
        private Scalar upperPurple2 = new Scalar(179, 255, 255);

        @Override
        public Mat processFrame(Mat input) {

            boolean detectedAnyColor = false;

            // Convert to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            // Convert to Gray
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.medianBlur(gray, gray, 5);

            // Detect circles
            Imgproc.HoughCircles(
                    gray,
                    circles,
                    Imgproc.HOUGH_GRADIENT,
                    1.2,
                    50,
                    100,
                    30,
                    90,
                    250
            );

            if (circles.cols() == 0) {
                telemetry.addData("Culoare", "nu exista");
                telemetry.update();
                return input;
            }

            // ========= LOOP THROUGH CIRCLES =========
            for (int i = 0; i < circles.cols(); i++) {

                double[] c = circles.get(0, i);
                int cx = (int) c[0];
                int cy = (int) c[1];
                int r  = (int) c[2];

                // Draw circle
                Imgproc.circle(input, new org.opencv.core.Point(cx, cy), r, new Scalar(0,255,255), 2);

                // ROI bounds
                int x = Math.max(cx - r, 0);
                int y = Math.max(cy - r, 0);
                int w = Math.min(r * 2, input.width()  - x);
                int h = Math.min(r * 2, input.height() - y);

                roiHSV = hsv.submat(new Rect(x, y, w, h));

                // Masks
                Core.inRange(roiHSV, lowerGreen, upperGreen, maskGreen);
                Core.inRange(roiHSV, lowerPurple1, upperPurple1, maskPurple1);
                Core.inRange(roiHSV, lowerPurple2, upperPurple2, maskPurple2);

                Core.add(maskPurple1, maskPurple2, maskPurple);

                int greenPixels  = Core.countNonZero(maskGreen);
                int purplePixels = Core.countNonZero(maskPurple);

                String color;

                if (greenPixels == 0 && purplePixels == 0) {
                    color = "nu exista";
                } else if (greenPixels > purplePixels) {
                    color = "VERDE";
                    detectedAnyColor = true;
                } else {
                    color = "MOV";
                    detectedAnyColor = true;
                }

                Imgproc.putText(
                        input,
                        color,
                        new org.opencv.core.Point(cx - r, cy - r - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        new Scalar(255, 255, 255),
                        2
                );

                telemetry.addData("Sferă " + i, color);
            }

            if (!detectedAnyColor) {
                telemetry.addData("Culoare", "nu exista");
            }

            telemetry.update();
            return input;
        }
    }
}
