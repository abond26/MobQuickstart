package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret;

import com.qualcomm.robotcore.hardware.Servo;

public interface TurretConstants {
    double shooterP = 200;
    double shooterF = 19.8;
    double ROTATOR_ZERO_POS = 0.5;
    double rotator180RangePos = 0.56;

    double CLOSE_VELOCITY = 1300;
    double MID_VELOCITY = 1500;
    double FAR_VELOCITY = 1800;
    double SCALE_RANGE_LOWER = 0.011;
    double SCALE_RANGE_UPPER = 0.5828;
    //
}