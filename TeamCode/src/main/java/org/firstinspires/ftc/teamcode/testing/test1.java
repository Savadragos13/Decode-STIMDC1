package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp(name = "IMU Test - Yaw Pitch Roll", group = "Test")
public class test1 extends LinearOpMode {

    IMU imu;

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);

        telemetry.addLine("IMU initializat");
        telemetry.addLine("Apasa START");
        telemetry.update();

        waitForStart();

        // Reseteaza yaw la 0
        imu.resetYaw();

        while (opModeIsActive()) {

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            telemetry.addData("Yaw (Heading)", "%.2f", angles.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", "%.2f", angles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", "%.2f", angles.getRoll(AngleUnit.DEGREES));

            telemetry.update();
        }
    }
}
