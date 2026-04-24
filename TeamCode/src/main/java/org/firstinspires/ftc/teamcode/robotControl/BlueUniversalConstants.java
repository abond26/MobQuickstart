package org.firstinspires.ftc.teamcode.robotControl;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.geometry.Pose;
@Configurable
public interface BlueUniversalConstants {
    static double rpmincrement = 20;
    double hoodIncrement = 0.005;
    double rotatorIncrement = 0.00002; // scaled for 5-turn servo (was 0.0001 for standard 180° servo)
    int PIPELINENUM = 1;
    // Pose defaultPose = new Pose(24.4, 126.7, Math.toRadians(143));
    Pose defaultPose = new Pose(65.15832205683355, 8.779431664411355, Math.toRadians(90 ));
    Pose target = new Pose(4, 144, Math.toRadians(144));
    Pose autonTarget = new Pose(0,144, Math.toRadians(144));
    Pose aprilTagPose = new Pose(16, 132, Math.toRadians(145));

    Double dpadUpHeading = 0.0;

    double CLOSE_HOOD_POSITION = 1; // Hood position for close shots
    double MID_HOOD_POSITION = 0.7;
    double FAR_HOOD_POSITION = 0;

}
