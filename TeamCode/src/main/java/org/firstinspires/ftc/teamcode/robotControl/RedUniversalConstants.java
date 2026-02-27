package org.firstinspires.ftc.teamcode.robotControl;

import com.pedropathing.geometry.Pose;

public interface RedUniversalConstants {
    double launcherSpeedIncrement = 20;
    double hoodIncrement = 0.005;
    int rotatorIncrement = 10; //10 is for testing, 50 for real game
    int PIPELINENUM = 1;
    //Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose defaultPose = new Pose(26.7, 132, Math.toRadians(145));
    Pose target = new Pose(141, 144, 0);


    double CLOSE_HOOD_POSITION = 1; // Hood position for close shots
    double MID_HOOD_POSITION = 0.5;
    double FAR_HOOD_POSITION = 0;
}
