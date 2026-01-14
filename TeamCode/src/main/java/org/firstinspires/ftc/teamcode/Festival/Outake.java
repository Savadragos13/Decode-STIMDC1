package org.firstinspires.ftc.teamcode.Festival;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "o1")
public class Outake  extends OpMode{

    DcMotor motor1, motor2;
    @Override
    public void init () {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor1.setDirection(DcMotor.Direction.FORWARD);

        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor2.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop () {
        motor1.setPower(0.9);
        motor2.setPower(0.9);
    }

}
