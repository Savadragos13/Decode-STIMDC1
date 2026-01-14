package org.firstinspires.ftc.teamcode.senzori;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name ="Senzor_sortare")
public class Senzor_de_culoare extends OpMode{

    ColorSensor color;

    @Override
    public void init(){
        color = hardwareMap.get(ColorSensor.class, "color");
    }

    @Override
    public void loop(){
        int red = color.red();
        int green = color.green();
        int blue = color.blue();

        String detectedColor = "NONE";

        if (green > red && green > blue &&  green > 100){
            detectedColor = "GREEN";
        }else if (red > 100 && blue > 100 && green < red && green<blue){
            detectedColor = "PURPLE";
        }

        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        telemetry.addData("Detected", detectedColor);
        telemetry.update();
    }

}
