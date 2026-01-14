package org.firstinspires.ftc.teamcode.Programeprincipale;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "iultim")
public class Teleop_albastru extends OpMode{
    ColorSensor color;
    DcMotor motor1out, motor2out;
    DcMotor motorin;
    DcMotor motorContinuu;
    Servo unghi, aruncare;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    final int TARGET_TAG_ID = 20;

    final double TARGET_DISTANCE = 12.0; // inch
    final double X_TOLERANCE = 1.0;
    final double Z_TOLERANCE = 2.0;
    final double TURN_GAIN = 0.03;
    final double DRIVE_GAIN = 0.04;
    final double MAX_POWER = 0.35;

    double d, y = 1.2, y0 = 0.2, v = 15, u;

    // Store detected ID
    int detectedTagID = -1;

    int red = color.red();
    int green = color.green();
    int blue = color.blue();

    private double clip(double value) {
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, value));
    }

    public static double computeAngle(double d, double v, double k, double y0) {

        double g = 9.81;

        // (d / v)^2
        double dvSquared = Math.pow(d / v, 2);

        // C = 1/2 * g * (d/v)^2
        double C = 0.5 * g * dvSquared;

        // A = C * (k - y0) - d^2
        double A = C * (k - y0) - Math.pow(d, 2);

        // B = k^2 + y0^2 + d^2 - 2*k*y0
        double B = Math.pow(k, 2)
                + Math.pow(y0, 2)
                + Math.pow(d, 2)
                - 2 * k * y0;

        // Discriminant
        double discriminant = Math.pow(A, 2)
                - 4 * B * Math.pow(C, 2);

        if (discriminant < 0) {
            throw new IllegalArgumentException("Negative discriminant");
        }

        double fraction =
                (-A + Math.sqrt(discriminant))
                        / (2 * B);

        if (fraction < 0 || fraction > 1) {
            throw new IllegalArgumentException("acos argument out of range: " + fraction);
        }

        fraction = Math.max(0.0, Math.min(1.0, fraction));

        // acos(sqrt(...)) → radians → degrees
        return Math.toDegrees(Math.acos(Math.sqrt(fraction)));
    }


    @Override
    public void init(){
        color = hardwareMap.get(ColorSensor.class, "color");

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .build();

        unghi = hardwareMap.get(Servo.class, "s0");
        aruncare = hardwareMap.get(Servo.class, "s1");

        motor1out = hardwareMap.get(DcMotor.class, "motor1out");
        motor2out = hardwareMap.get(DcMotor.class, "motor2out");
        motorin = hardwareMap.get(DcMotor.class, "motorin");
        motorContinuu = hardwareMap.get(DcMotor.class, "motorcontinuu");
    }

    @Override
    public void loop(){

        // ---------- COLOR DETECTION ----------

        String detectedColor = "NONE";

        if (green > red && green > blue &&  green > 100){
            detectedColor = "GREEN";
        }else if (red > 100 && blue > 100 && green < red && green<blue){
            detectedColor = "PURPLE";
        }

        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        telemetry.addData("Detected", detectedColor);

        // ---------- APRILTAG DETECTION ----------

        detectedTagID = -1;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        AprilTagDetection targetTag = null;
        for (AprilTagDetection tag : detections) {
            if (tag.id == TARGET_TAG_ID) {
                detectedTagID = tag.id;
                targetTag = tag;
                d = tag.ftcPose.z;
                break; // găsit tag-ul dorit, nu mai căutăm
            }
        }


        if (detectedTagID != -1) {
            telemetry.addData("AprilTag ID", detectedTagID);
        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.update();

        if (gamepad1.right_bumper && targetTag != null) {

            double x = targetTag.ftcPose.x;
            double z = targetTag.ftcPose.z;

            // Calculează putere proportională
            double turnPower = clip(x * TURN_GAIN);
            double drivePower = clip((z - TARGET_DISTANCE) * DRIVE_GAIN);

            telemetry.addLine("AUTO-CENTER ACTIVE");
            telemetry.addData("Turn Power", turnPower);
            telemetry.addData("Drive Power", drivePower);

            // Optional: trimite la motoare
            // leftDrive.setPower(drivePower + turnPower);
            // rightDrive.setPower(drivePower - turnPower);

            // Stop dacă e aproape de centru
            if (Math.abs(x) < X_TOLERANCE && Math.abs(z - TARGET_DISTANCE) < Z_TOLERANCE) {
                // leftDrive.setPower(0);
                // rightDrive.setPower(0);
                telemetry.addLine("Tag centered!");
            }
        }

        // ---------- CONTROL ----------

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        float mo = gamepad2.right_trigger, mi = gamepad2.left_trigger;
        if (Math.abs(mo) > 0.1) {
            motor1out.setPower(-1);
            motor2out.setPower(1);

        }
        else{
            motor1out.setPower(0);
            motor2out.setPower(0);
        }
        if (Math.abs(mi) > 0.1) {
            motorin.setPower(1);
        }
        else
            motorin.setPower(0);

        u = computeAngle(d, v, y, y0);

         if (gamepad2.x){
             unghi.setPosition(u/180);

         }
    }/*leftFront rightFront leftRear rightRear*/

}
