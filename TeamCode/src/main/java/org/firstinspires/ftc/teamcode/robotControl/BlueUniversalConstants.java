package org.firstinspires.ftc.teamcode.robotControl;

import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.geometry.Pose;

public interface BlueUniversalConstants {
    double launcherSpeedIncrement = 20;
    double hoodIncrement = 0.005;
    double rotatorIncrement = 0.00002; // scaled for 5-turn servo (was 0.0001 for standard 180° servo)
    int PIPELINENUM = 1;
    // Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose defaultPose = new Pose(25.448900388098327, 131.8809831824062, Math.toRadians(234));
    Pose target = new Pose(0, 144, Math.toRadians(144));
    Pose aprilTagPose = new Pose(16, 132, Math.toRadians(145));
    // Pose dpadUpPose = new Pose(130.8, 24.08, Math.toRadians(0)); //change for
    // every auton close
    Double dpadUpHeading = 0.0;
    //

    double CLOSE_HOOD_POSITION = 1; // Hood position for close shots
    double MID_HOOD_POSITION = 0.7;
    double FAR_HOOD_POSITION = 0;
    double CLOSE_VELO = 1100; // Hood position for close shots
    double MID_VELO = 1350;
    double FAR_VELO = 1600;
    double Fix = 1.0;

    // ── Localization Constants ──
    double METERS_TO_INCHES = 39.3701;
    double FIELD_OFFSET_X = 72.0;
    //
    double FIELD_OFFSET_Y = 72.0;
}
