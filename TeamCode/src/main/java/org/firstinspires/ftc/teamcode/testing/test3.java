package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "testing3", group = "Testing")
public class test3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double forwardVel = 0.0;   // in/s (simulat)
        double angularVel = 0.0;   // rad/s

        double step = 1.0;         // cât crește pe secundă

        waitForStart();

        while (opModeIsActive()) {

            // crește viteza înainte
            if (gamepad1.left_stick_y < -0.5) {
                forwardVel += step;
            } else if (gamepad1.left_stick_y > 0.5) {
                forwardVel -= step;
            }

            // crește viteza de rotație
            if (gamepad1.right_stick_x > 0.5) {
                angularVel += Math.toRadians(5);
            } else if (gamepad1.right_stick_x < -0.5) {
                angularVel -= Math.toRadians(5);
            }

            // LIMITĂ DE SIGURANȚĂ
            forwardVel = Math.max(-60, Math.min(60, forwardVel));
            angularVel = Math.max(-Math.toRadians(180), Math.min(Math.toRadians(180), angularVel));

            // aplicăm mișcarea
            drive.setWeightedDrivePower(
                    new com.acmerobotics.roadrunner.geometry.Pose2d(
                            forwardVel / 60.0,   // normalizat
                            0,
                            angularVel / Math.toRadians(180)
                    )
            );

            telemetry.addLine("=== MAX VELOCITY TEST ===");
            telemetry.addData("Forward Velocity (in/s)", forwardVel);
            telemetry.addData("Angular Velocity (deg/s)", Math.toDegrees(angularVel));
            telemetry.addLine("Left stick Y = forward/back");
            telemetry.addLine("Right stick X = rotate");
            telemetry.addLine("Cauta limita unde robotul devine instabil");
            telemetry.update();

            sleep(100);
        }
    }
}
