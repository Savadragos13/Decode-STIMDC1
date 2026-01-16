package org.firstinspires.ftc.teamcode.OA;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;
/* PLAN AFACERI
    -Plecam de la rosu, triunghiul mare, robotul drept si pe mijloc cu linia alba a triunghiului, lipit de cos
    -Mergem inainte 51 in = ~130 cm ,vedem daca aruncam (Trebuie mai mult)
    -Se intoarce 135 grade
    -Merge inainte, iar apoi merge din nou inainte dar mai lent pentru a lua cele 3 bile de pe prima linie
    -Merge cu spatele, se intoarce -135 grade
    -Arunca bilele
 */




@Config
@Autonomous(name= "OARedBig")
public class OARedBig extends LinearOpMode {
    public static double a = 54, c=40; // in
    public static double b = 135; //grade

    DcMotor motor1, motor2, motor3, motor4;

    VoltageSensor batteryVoltageSensor;
    static final double V_REF = 11;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(51)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(140))
                .setConstraints (new MecanumVelocityConstraint(10, TRACK_WIDTH), // Viteza maximă dorită (în loc de 10, am pus 30 ca exemplu)
                new ProfileAccelerationConstraint(30)           // Accelerația maximă dorită
    )
                .lineToConstantHeading(new Vector2d(23,33))
                .resetConstraints()
                .waitSeconds(3)
                .back(42)
                .turn(Math.toRadians(-140))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(traj1);

            motor3.setPower(compensatedPower(-0.6));
            motor4.setPower(compensatedPower(0.6));
            sleep(2000);
            motor1.setPower(-0.75);
            motor2.setPower(-0.75);
            sleep(3000);
            motor3.setPower(0);
            motor4.setPower(0);
            motor1.setPower(0.5);
            motor2.setPower(0.5);
            sleep(2000);
            motor1.setPower(-0.5);
            motor2.setPower(-0.1);
            drive.followTrajectorySequence(traj2);
            motor3.setPower(compensatedPower(-0.6));
            motor4.setPower(compensatedPower(0.6));
            sleep(2000);
            motor1.setPower(-0.75);
            motor2.setPower(-0.75);
            sleep(4500);

            motor3.setPower(0);
            motor4.setPower(0);
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
    double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    double compensatedPower(double powerDorita) {
        double voltage = batteryVoltageSensor.getVoltage();
        double power = powerDorita * (V_REF / voltage);
        telemetry.addData("voltaj", voltage);
        return clip(power, -1.0, 1.0);
    }


}
