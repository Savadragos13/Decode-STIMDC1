package org.firstinspires.ftc.teamcode.camera;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.videoio.VideoCapture;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org. openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.MatOfPoint;
import java.util.ArrayList;
import java.util.List;


@TeleOp(name="Camera_2", group = "Paste")
public class Test2 extends LinearOpMode {
    private OpenCvWebcam webcam;//initializarea camerei
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());//Pentru a vizualiza camera in Drive station
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam") , cameraMonitorViewId);//Creem obiectul webcam conectandu-l la camera definita in configuratia hardwerw "Webcam"
        webcam.setPipeline(new SamplePipeline());//legam camera la clasa SamplePipeline , care va procesa imaginea
        webcam.openCameraDevice();//deschide camera
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);//incepa transmisia cu rezolutia 640x480
        telemetry.addData("Status", "Camera initializata");//verificarea starea programului
        telemetry.update();
        waitForStart();//programul asteapta pana utilizatorul apasa start in Drive Station

        while (opModeIsActive()){
            telemetry.addData("Striming", "Camera activata");
            telemetry.update();
            sleep(100);
        }//cat timp OPModeisadctive codul afiseaza mesaje si mentine transmitera camarei
    }
    class SamplePipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            boolean detectedAnyColor = false;   // verificăm dacă detectăm vreo culoare

            // Convert to HSV and grayscale
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.medianBlur(gray, gray, 5);

            // --- DETECTARE SFERE (HOUGH CIRCLES) ---
            Mat circles = new Mat();
            Imgproc.HoughCircles(
                    gray,
                    circles,
                    Imgproc.HOUGH_GRADIENT,
                    1.2,
                    50,
                    100,
                    30,
                    65,
                    250
            );

            // --- INTERVALE VERDE ---
            Scalar lowerGreen = new Scalar(50, 100, 100);
            Scalar upperGreen = new Scalar(70, 255, 255);

            // --- INTERVALE MOV ---
            Scalar lowerPurple1 = new Scalar(129, 80, 80);
            Scalar upperPurple1 = new Scalar(158, 255, 255);

            Scalar lowerPurple2 = new Scalar(158, 80, 80);
            Scalar upperPurple2 = new Scalar(179, 255, 255);

            // Dacă NU există cercuri → NU EXISTĂ
            if (circles.cols() == 0) {
                telemetry.addData("Culoare", "nu exista");
                telemetry.update();
                return input;
            }

            // --- PARCURGEM FIECARE SFERĂ ---
            for (int i = 0; i < circles.cols(); i++) {

                double[] circle = circles.get(0, i);
                int centerX = (int) circle[0];
                int centerY = (int) circle[1];
                int radius = (int) circle[2];

                // Desenăm cercul găsit
                Imgproc.circle(input, new org.opencv.core.Point(centerX, centerY), radius, new Scalar(0, 255, 255), 2);

                // Creăm ROI pentru interiorul sferei
                Rect roiRect = new Rect(
                        Math.max(centerX - radius, 0),
                        Math.max(centerY - radius, 0),
                        Math.min(radius * 2, input.width() - (centerX - radius)),
                        Math.min(radius * 2, input.height() - (centerY - radius))
                );

                Mat roiHSV = new Mat(hsv, roiRect);

                // Măști culoare
                Mat maskGreen = new Mat();
                Mat maskPurple1 = new Mat();
                Mat maskPurple2 = new Mat();
                Mat maskPurple = new Mat();

                Core.inRange(roiHSV, lowerGreen, upperGreen, maskGreen);
                Core.inRange(roiHSV, lowerPurple1, upperPurple1, maskPurple1);
                Core.inRange(roiHSV, lowerPurple2, upperPurple2, maskPurple2);
                Core.add(maskPurple1, maskPurple2, maskPurple);

                int greenPixels = Core.countNonZero(maskGreen);
                int purplePixels = Core.countNonZero(maskPurple);

                String color;

                if (greenPixels == 0 && purplePixels == 0) {
                    color = "nu exista";  // nu s-a găsit culoare în interiorul sferei
                } else if (greenPixels > purplePixels) {
                    color = "VERDE";
                    detectedAnyColor = true;
                } else {
                    color = "MOV";
                    detectedAnyColor = true;
                }

                // Afisare text
                Imgproc.putText(
                        input,
                        color,
                        new org.opencv.core.Point(centerX - radius, centerY - radius - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        new Scalar(255, 255, 255),
                        2
                );

                telemetry.addData("Sferă " + i, color);
            }

            // Dacă am detectat cercuri dar niciuna nu are culoare validă
            if (!detectedAnyColor) {
                telemetry.addData("Culoare", "nu exista");
            }

            telemetry.update();
            return input;
        }
    }



}



