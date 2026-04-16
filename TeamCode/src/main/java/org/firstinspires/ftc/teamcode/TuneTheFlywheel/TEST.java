package org.firstinspires.ftc.teamcode.TuneTheFlywheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TEST extends OpMode {

    Flywheel flywheel = new Flywheel();
    public double kV = 0.0001;
    public double kS = 0.145;
    public double goalRPM = 1700;
    double[] incriments = { 0.000001, 0.00001, 0.0001, 0.001, 0.01 };
    int incrementIdx = 4;
    double TargetRpm = 1000;

    @Override
    public void init() {
        flywheel.init(hardwareMap);

    }

    @Override
    public void loop() {
        flywheel.setMotorRPM(TargetRpm);

        if (gamepad1.a) {
            TargetRpm = 2000;
        } else if (gamepad1.b) {
            TargetRpm = 1000;
        }
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("GOAL", "%.6f", TargetRpm);
    }

}
