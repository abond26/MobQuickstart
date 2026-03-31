package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;



@TeleOp(name = "Color Sensor Tuner", group = "Tests")
public class ColorSensorTunerOpMode extends LinearOpMode {
    
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Initialize the color sensor from the hardware map
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        telemetry.addLine("Ready to start. Press Play to see color values.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            
            float normRed = colors.red / colors.alpha;
            float normGreen = colors.green / colors.alpha;
            float normBlue = colors.blue / colors.alpha;

            telemetry.addLine("put ball in");
            telemetry.addLine("update sensors based off");
            telemetry.addLine("---------------------------------------------");
            telemetry.addData("Red", "%.3f", normRed);
            telemetry.addData("Green", "%.3f", normGreen);
            telemetry.addData("Blue", "%.3f", normBlue);

            // Helpful indicator for the absolute dominant color
            if (normRed > normBlue && normRed > normGreen) {
                telemetry.addLine("more of red");
            } else if (normGreen > normRed && normGreen > normBlue) {
                telemetry.addLine("more of green");
            } else if (normBlue > normRed && normBlue > normGreen) {
                telemetry.addLine("Dore of blue");
            }

            telemetry.update();
        }
    }
}
