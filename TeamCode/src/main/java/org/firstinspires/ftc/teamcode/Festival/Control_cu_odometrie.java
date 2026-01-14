
package org.firstinspires.ftc.teamcode.Festival;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = " Control_cu_odometrie")
public class Control_cu_odometrie extends LinearOpMode{
    DcMotor motorout;
    DcMotor motorin;

    CRServo impins;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        motorout = hardwareMap.get(DcMotor.class, "motor1out");
        motorin = hardwareMap.get(DcMotor.class, "motorin");
        impins = hardwareMap.get(CRServo.class, "s");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
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
                motorout.setPower(-0.99);
            }
            else{
                motorout.setPower(0);
            }
            if (Math.abs(mi) > 0.1) {
                motorin.setPower(1);
            }
            else
                motorin.setPower(0);
            if (gamepad2.x){
                impins.setPower(1);
            } else if (gamepad2.a){
                impins.setPower(-1);
            } else {
                impins.setPower(0);
            }
        }/*leftFront rightFront leftRear rightRear*/
    }
}
