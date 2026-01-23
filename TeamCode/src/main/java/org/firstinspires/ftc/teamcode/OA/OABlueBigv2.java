package org.firstinspires.ftc.teamcode.OA;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
/// plan de afaceri
/// merg 59.11inch, fac stanga, iau bile si arunc
@Config
@Autonomous(name = "OABlueBigv2")
public class OABlueBigv2 extends LinearOpMode {

    DcMotor motor1, motor2, motor3, motor4;
    VoltageSensor batteryVoltageSensor;

    static final double V_REF = 11.0;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        Pose2d startPose = new Pose2d(0, 0, 0);

        telemetry.addLine("Ce faci,Sebi?");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPose);
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(59.1128326716)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-139))
                .setConstraints(
                        new MecanumVelocityConstraint(8, TRACK_WIDTH),
                        new ProfileAccelerationConstraint(20)
                )
                .forward(40)
                .resetConstraints()
                .waitSeconds(2)
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(59.1128326716, 0,139))
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .strafeRight(17)
                .build();
        motor3.setPower(compensatedPower(-0.58));
        motor4.setPower(compensatedPower(0.58));
        drive.followTrajectory(traj1);
        motor1.setPower(compensatedPower(-0.75));
        motor2.setPower(compensatedPower(-0.75));
        sleep(2500);
        motor3.setPower(0);
        motor4.setPower(0);
        motor1.setPower(0.5);
        motor2.setPower(0.5);
        sleep(1000);
        motor1.setPower(-0.5);
        motor2.setPower(-0.1);
        sleep(1500);
        drive.followTrajectorySequence(traj2);
        motor3.setPower(compensatedPower(-0.58));
        motor4.setPower(compensatedPower(0.58));
        sleep(1000);
        motor1.setPower(0);
        motor2.setPower(0);
        drive.followTrajectorySequence(traj3);
        motor1.setPower(compensatedPower(-0.75));
        motor2.setPower(compensatedPower(-0.75));
        sleep(2500);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        drive.followTrajectorySequence(traj4);
        sleep(1000);
    }
    double compensatedPower(double powerDorita) {
        double voltage = batteryVoltageSensor.getVoltage();
        double power = powerDorita * (V_REF / voltage);
        return Math.max(-1.0, Math.min(1.0, power));
    }
}
