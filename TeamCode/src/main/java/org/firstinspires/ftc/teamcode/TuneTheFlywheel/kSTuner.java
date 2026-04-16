package org.firstinspires.ftc.teamcode.TuneTheFlywheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class kSTuner extends OpMode {

    Flywheel flywheel = new Flywheel();
    public double kS = 0.1;
    double[] incriments = { 0.000001, 0.00001, 0.0001, 0.001, 0.01 };
    int incrementIdx = 4;

    @Override
    public void init() {
        flywheel.init(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed() && incrementIdx < 4) {
            incrementIdx++;
        } else if (gamepad1.dpadLeftWasPressed() && incrementIdx > 0) {
            incrementIdx--;
        }

        double currentStep = incriments[incrementIdx];

        // change kS
        if (gamepad1.dpadUpWasPressed()) {
            kS += currentStep;
        }
        if (gamepad1.dpadDownWasPressed()) {
            kS -= currentStep;
        }

        flywheel.setMotorPower(kS);

        telemetry.addData("kS", "%.6f", kS);
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("Ticks per Sec", flywheel.getTicksPerSec());

    }
}
