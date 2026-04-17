package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tests.ColorTesting;

public interface IntakeConstants {
    ElapsedTime colorTimer = new ElapsedTime();
    ColorTesting.DetectedColor currentlyDetectedColor = ColorTesting.DetectedColor.UNKNOWN;
    //how long should i hold this b all
    public double requiredDetectionTimeSeconds = 0.3;
    //
     //
}
