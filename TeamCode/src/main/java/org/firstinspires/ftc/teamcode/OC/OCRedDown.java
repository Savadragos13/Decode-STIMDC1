
package org.firstinspires.ftc.teamcode.OC;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="OCRedDown")

public class OCRedDown extends LinearOpMode {
    DcMotor motorout;
    DcMotor motorin;
    DcMotor motorContinuu;
    Servo unghi;
    CRServo impins, belt;

    @Override
    public void runOpMode() throws InterruptedException {
        motorout = hardwareMap.get(DcMotor.class, "motor1out");
        motorin = hardwareMap.get(DcMotor.class, "motorin");
        impins = hardwareMap.get(CRServo.class, "s");
        belt = hardwareMap.get(CRServo.class, "s2");


        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0); // 1. Set Starting Position
        drive.setPoseEstimate(startPose); // 2. Set Initial Pose

        // 3. Build Trajectories
        Trajectory StartTrajectory = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-9, 0, Math.toRadians(45)))
                .build();

        Trajectory ToPickupFromShootTrajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(46, 0, Math.toRadians(60)))
                .build();

        Trajectory ToShootFromPickupTrajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-46, 0, Math.toRadians(-60)))
                .build();

        if(isStopRequested()) return;

        // 4. Follow Trajectories
        drive.followTrajectory(StartTrajectory);
        DoOuttake();
        //drive.followTrajectory(ToPickupFromShootTrajectory);
        //sleep(2000);
        //motorin.setPower(1);
        //drive.followTrajectory(ToShootFromPickupTrajectory);
        //motorin.setPower(0);
        //DoOuttake();
        //drive.followTrajectory(ToPickupFromShootTrajectory);
        //sleep(2000);
        //motorin.setPower(1);
        //drive.followTrajectory(ToShootFromPickupTrajectory);
        //DoOuttake();

        return;
    }

    public void DoOuttake()
    {
        motorout.setPower(1);
        belt.setPower(-1);
        impins.setPower(1);
        sleep(500);
        impins.setPower(0);
        belt.setPower(0);
        motorout.setPower(0);
    }
}




