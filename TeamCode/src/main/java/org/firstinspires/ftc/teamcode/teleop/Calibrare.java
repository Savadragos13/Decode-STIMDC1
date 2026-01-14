package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "Calibrare")
public class Calibrare extends LinearOpMode {

    DcMotor motor1, motor2, motor3, motor4;
    double TICKS_PER_REV = 28 * 19.2; // = 537.6



    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor1.setDirection(DcMotor.Direction.FORWARD);

        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor3.setDirection(DcMotor.Direction.FORWARD);

        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motor4.setDirection(DcMotor.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);
        motor4.setPower(1);
        sleep(60000); // run for 1 minute
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        telemetry.addData("Encoder count1: ", motor1.getCurrentPosition());
        telemetry.addData("Encoder count2: ", motor2.getCurrentPosition());
        telemetry.addData("Encoder count3: ", motor3.getCurrentPosition());
        telemetry.addData("Encoder count4: ", motor4.getCurrentPosition());
        telemetry.update();
        sleep(1000000); // run for 1000 seconds

    }

}
