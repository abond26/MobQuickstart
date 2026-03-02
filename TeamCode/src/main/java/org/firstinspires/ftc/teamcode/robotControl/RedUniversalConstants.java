package org.firstinspires.ftc.teamcode.robotControl;

import com.pedropathing.geometry.Pose;

public interface RedUniversalConstants {
    double launcherSpeedIncrement = 20;
    double hoodIncrement = 0.005;
    int rotatorIncrement = 10; //10 is for testing, 50 for real game
    int PIPELINENUM = 0;
    //Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose defaultPose = new Pose(117.3, 132, Math.toRadians(36));
    Pose target = new Pose(140, 144, 0);
    Pose aprilTagPose = new Pose(128, 132, Math.toRadians(145));
    Pose dpadUpPose = new Pose(15.77, 14.84, Math.toRadians(180));


    double CLOSE_HOOD_POSITION = 1; // Hood position for close shots
    double MID_HOOD_POSITION = 0.5;
    double FAR_HOOD_POSITION = 0;
}
