package org.firstinspires.ftc.teamcode.Festival;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Comtrol_fara_odometrie")
public class Control_fara_odometrie  extends OpMode{

        DcMotor motor1, motor2, motor3, motor4;
        DcMotor motor1out, motor2out;
        DcMotor motorin;
        DcMotor motorContinuu;
        @Override
        public void init () {
            motor1 = hardwareMap.get(DcMotor.class, "motor1");
            motor1.setDirection(DcMotor.Direction.FORWARD);

            motor2 = hardwareMap.get(DcMotor.class, "motor2");
            motor2.setDirection(DcMotor.Direction.REVERSE);

            motor3 = hardwareMap.get(DcMotor.class, "motor3");
            motor3.setDirection(DcMotor.Direction.FORWARD);

            motor4 = hardwareMap.get(DcMotor.class, "motor4");
            motor4.setDirection(DcMotor.Direction.REVERSE);

            motor1out = hardwareMap.get(DcMotor.class, "motor1out");
            motor2out = hardwareMap.get(DcMotor.class, "motor2out");
            motorin = hardwareMap.get(DcMotor.class, "motorin");
            motorContinuu = hardwareMap.get(DcMotor.class, "motorcontinuu");
        }

        @Override
        public void loop () {
            float x = -gamepad1.left_stick_x;
            float y = -gamepad1.left_stick_y;
            float i = gamepad1.right_stick_x;
            merge(x, y, i);
            motorContinuu.setPower(1);

            float mo = gamepad2.right_trigger, mi = gamepad2.left_trigger;
            if (Math.abs(mo) > 0.1) {
                motor1out.setPower(1);
                motor2out.setPower(1);
            }
            if (Math.abs(mi) > 0.1) {
                motorin.setPower(1);
            }

        }

        public void merge ( float x, float y, float z){
            double frontLeftPower = x + y + z;
            double frontRightPower = x - y - z;
            double backLeftPower = x - y + z;
            double backRightPower = x + y - z;
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
