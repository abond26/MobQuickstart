package org.firstinspires.ftc.teamcode.robotControl;

import com.pedropathing.geometry.Pose;

public interface BlueUniversalConstants {
    double launcherSpeedIncrement = 20;
    double hoodIncrement = 0.005;
    int rotatorIncrement = 10; //10 is for testing, 50 for real game
    int PIPELINENUM = 1;
    //Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose defaultPose = new Pose(26.7, 132, Math.toRadians(144));
    Pose target = new Pose(4, 144, 144);
    Pose aprilTagPose = new Pose(16, 132, Math.toRadians(145));
    //Pose dpadUpPose = new Pose(130.8, 24.08, Math.toRadians(0)); //change for every auton close
    Pose dpadUpPose = new Pose(122.5, 12.5, Math.toRadians(0)); //change for every auton far




    double CLOSE_HOOD_POSITION = 1; // Hood position for close shots
    double MID_HOOD_POSITION = 0.5;
    double FAR_HOOD_POSITION = 0;
    double CLOSE_VELO = 1100; // Hood position for close shots
    double MID_VELO = 1350;
    double FAR_VELO = 1600;

    // ── Localization Constants ──
    double METERS_TO_INCHES = 39.3701;
    double FIELD_OFFSET_X = 72.0;
    double FIELD_OFFSET_Y = 72.0;
}
