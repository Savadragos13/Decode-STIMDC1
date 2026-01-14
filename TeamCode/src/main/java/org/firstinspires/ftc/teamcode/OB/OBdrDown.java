package org.firstinspires.ftc.teamcode.OB;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="OBdrDown")

public class OBdrDown extends LinearOpMode {

    DcMotor motor1, motor2, motor3, motor4;

    static final double ticks = 288.0;
    double newTarget;
    double x=0.4,a = 0.84, b=0.33;

    @Override
    public void runOpMode() {


        motor1 = hardwareMap.get(DcMotor.class, "leftFront");
        motor1.setDirection(DcMotor.Direction.FORWARD);

        motor2 = hardwareMap.get(DcMotor.class, "rightFront");
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor3 = hardwareMap.get(DcMotor.class, "leftRear");
        motor3.setDirection(DcMotor.Direction.FORWARD);

        motor4 = hardwareMap.get(DcMotor.class, "rightRear");
        motor4.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            pas0(x);
           /* pas1(a); ///ma intorc 30 grade la stanga
           // pas2();  ///arunc bilele
          /*  pas1(-a);/// ma intorc la -30 grade la dreapta
            pas3(b); ///merge spre dreapta, stau 7 secunde
            pas4(-b);///merge stanga
            pas1(a); ///ma intorc 30 grade la stanga
          //  pas2();  ///arunc bilele
            pas1(-a);/// ma intorc la -30 grade la dreapta
            pas3(b); ///merge spre dreapta, stau 7 secunde*/
        }


    }
    public void pas0(double turnage) {

        if (turnage != 0) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newTarget = ticks / turnage;
            motor1.setTargetPosition((int) -newTarget);
            motor1.setPower(0.4);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setTargetPosition((int) -newTarget);
            motor2.setPower(0.4);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setTargetPosition((int) -newTarget);
            motor3.setPower(0.4);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor4.setTargetPosition((int) -newTarget);
            motor4.setPower(0.4);
            motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(4000);
        }
    }
    public void pas1(double turnage) {

        if (turnage != 0) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newTarget = ticks / turnage;
            motor1.setTargetPosition((int) newTarget);
            motor1.setPower(0.4);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setTargetPosition((int) newTarget);
            motor2.setPower(0.4);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setTargetPosition((int) newTarget);
            motor3.setPower(-0.4);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor4.setTargetPosition((int) newTarget);
            motor4.setPower(-0.4);
            motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(4000);
        }
    }

    public void pas2() {

        //motor1out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor1out.setPower(0.34);
        sleep(3000);

    }
    public void pas3(double turnage) {
        if (turnage != 0) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newTarget = ticks / turnage;
            motor1.setTargetPosition((int) -newTarget);
            motor1.setPower(0.4);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setTargetPosition((int) newTarget);
            motor2.setPower(0.4);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setTargetPosition((int) -newTarget);
            motor3.setPower(-0.4);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor4.setTargetPosition((int) newTarget);
            motor4.setPower(-0.4);
            motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(7000);
        }
    }

    public void pas4(double turnage) {
        if (turnage != 0) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newTarget = ticks / turnage;
            motor1.setTargetPosition((int) newTarget);
            motor1.setPower(0.4);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setTargetPosition((int) -newTarget);
            motor2.setPower(0.4);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setTargetPosition((int) -newTarget);
            motor3.setPower(-0.4);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor4.setTargetPosition((int) -newTarget);
            motor4.setPower(-0.4);
            motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(3000);
        }
    }


}