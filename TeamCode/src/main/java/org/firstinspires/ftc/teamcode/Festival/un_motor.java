package org.firstinspires.ftc.teamcode.Festival;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Unmotor")
public class un_motor  extends OpMode{

    DcMotor motor1;

    @Override
    public void init () {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor1.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop () {
        float x = -gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;
        float i = gamepad1.right_stick_x;

        float mo = gamepad2.right_trigger, mi = gamepad2.left_trigger;
        if (Math.abs(mo) > 0.1) {
            motor1.setPower(0.99);
        }


    }


}
