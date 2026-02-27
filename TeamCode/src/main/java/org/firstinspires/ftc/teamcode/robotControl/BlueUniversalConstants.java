package org.firstinspires.ftc.teamcode.robotControl;

import com.pedropathing.geometry.Pose;

public interface BlueUniversalConstants {
    double launcherSpeedIncrement = 20;
    double hoodIncrement = 0.005;
    int rotatorIncrement = 10; //10 is for testing, 50 for real game
    int PIPELINENUM = 0;
    //Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose defaultPose = new Pose(26.7, 132, Math.toRadians(144));
    Pose target = new Pose(2, 144, 0);
    Pose aprilTagPose = new Pose(16, 132, Math.toRadians(145));


    double CLOSE_HOOD_POSITION = 1; // Hood position for close shots
    double MID_HOOD_POSITION = 0.5;
    double FAR_HOOD_POSITION = 0;
}
