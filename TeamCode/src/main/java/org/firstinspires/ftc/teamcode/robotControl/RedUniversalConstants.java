package org.firstinspires.ftc.teamcode.robotControl;

import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.geometry.Pose;

public interface RedUniversalConstants {
    double launcherSpeedIncrement = 20;
    double hoodIncrement = 0.005;
    int rotatorIncrement = 10; //10 is for testing, 50 for real game
    int PIPELINENUM = 0;
    //Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose defaultPose = new Pose(117.3, 132, Math.toRadians(36));
    Pose target = new Pose(144, 144, Math.toRadians(36));
    Pose aprilTagPose = new Pose(16, 132, Math.toRadians(145));
    Double dpadUpHeading = 180.0;
    //


    double CLOSE_HOOD_POSITION = 1; // Hood position for close shots
    double MID_HOOD_POSITION = 0.5;
    //
    double FAR_HOOD_POSITION = 0;
    double CLOSE_VELO = 1100;
    double MID_VELO = 1350;
    double FAR_VELO = 1600;
    double Fix = 1.0;


    // ── Localization Constants ──
    double METERS_TO_INCHES = 39.3701;
    double FIELD_OFFSET_X = 72.0;
    double FIELD_OFFSET_Y = 72.0;
    //
}
