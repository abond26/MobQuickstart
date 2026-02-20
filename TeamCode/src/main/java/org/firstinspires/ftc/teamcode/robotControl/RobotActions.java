package org.firstinspires.ftc.teamcode.robotControl;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis.ChassisLocal;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision;

public class RobotActions {

    private ChassisLocal chassis;
    private Turret turret;
    private Intake intake;
    private TransferGate gate;
    private Vision vision;

    public RobotActions(ChassisLocal chassis,
                        Vision vision,
                        Turret turret,
                        TransferGate gate,
                        Intake intake) {

        this.chassis = chassis;
        this.vision = vision;
        this.turret = turret;
    }

    public void relocalize() {
        Pose pose = vision.getEstimatedPose();
        drive.setPoseEstimate(pose);
    }

    public void autoAdjustShooter() {
        double distance = vision.getTargetDistance();
        shooter.setVelocityFromDistance(distance);
    }
}
