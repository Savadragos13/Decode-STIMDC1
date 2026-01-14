package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name= "Italieni")
public class Test extends OpMode{
    DcMotor motor1,motor2,motor3,motor4 ;

    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor1.setDirection(DcMotor.Direction.FORWARD);

        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor3.setDirection(DcMotor.Direction.FORWARD);

        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motor4.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Hardwere:", "pornire");
    }

    public void loop() {
        float x=-gamepad1.left_stick_y;
        float y=gamepad1.left_stick_x;
        float i=gamepad1.right_stick_x;
        merge(x, y, i);

    }

    public void merge(float x, float y, float z) {
        double frontLeftPower    =  x +y +z;
        double frontRightPower   =  x -y -z;
        double backLeftPower     =  x -y +z;
        double backRightPower    =  x +y -z;
        double max = Math.max(Math.max(Math.max(frontLeftPower, backLeftPower), backRightPower), frontRightPower);
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }


        motor1.setPower(frontLeftPower);
        motor2.setPower(frontRightPower);
        motor3.setPower(backLeftPower);
        motor4.setPower(backRightPower);
    }
}
