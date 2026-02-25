package org.firstinspires.ftc.teamcode.robotControl;

import com.pedropathing.geometry.Pose;

public interface BlueUniversalConstants {
    double launcherSpeedIncrement = 20;
    double hoodIncrement = 0.005;
    int rotatorIncrement = 10; //10 is for testing, 50 for real game
    int PIPELINENUM = 0;
    //Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose target = new Pose(0, 144, 0);
}
