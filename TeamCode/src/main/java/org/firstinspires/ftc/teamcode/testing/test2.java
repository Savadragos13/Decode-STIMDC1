package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="testing2", group="Tuning")
public class test2 extends LinearOpMode {

    // --- CONSTANTE ROBOT ---
    private static final double WHEEL_RADIUS = 2.05; // inch
    private static final double MAX_RPM = 312;
    private static final double GEAR_RATIO = 1.0;

    // --- FEEDFORWARD INITIAL ---
    private double kV = 0.01488;
    private double kA = 0.00045;
    private double kStatic = 0.06749;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("=== Feedforward Tuner ===");
        telemetry.addLine("Use dpad_up/dpad_down to adjust kStatic");
        telemetry.addLine("Use dpad_right/dpad_left to adjust kA");
        telemetry.addLine("Press 'a' to test motion, 'b' to stop");
        telemetry.update();

        // --- Dummy mecanum drive pentru tuning ---
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // --- CONTROL KSTATIC ---
            if (gamepad1.dpad_up) {
                kStatic += 0.01;
            } else if (gamepad1.dpad_down) {
                kStatic -= 0.01;
            }

            // --- CONTROL KA ---
            if (gamepad1.dpad_right) {
                kA += 0.001;
            } else if (gamepad1.dpad_left) {
                kA -= 0.001;
            }

            // --- TEST MOTION ---
            double targetVelocity = 30.0; // inch/sec (poți schimba)
            double targetAccel = 10.0;    // inch/sec^2 (poți schimba)
            double power = kStatic + kV * targetVelocity + kA * targetAccel;

            if (gamepad1.a) { // pornește robotul
                drive.setMotorPowers(power, power, power, power);
            } else if (gamepad1.b) { // oprește robotul
                drive.setMotorPowers(0, 0, 0, 0);
            }

            // --- TELEMETRY ---
            telemetry.addData("kV", kV);
            telemetry.addData("kA", kA);
            telemetry.addData("kStatic", kStatic);
            telemetry.addData("Calculated Power", power);
            telemetry.addData("Instructions", "dpad_up/down=kStatic, dpad_left/right=kA, a=test, b=stop");
            telemetry.update();

            sleep(50); // 20 Hz update
        }
    }
}
