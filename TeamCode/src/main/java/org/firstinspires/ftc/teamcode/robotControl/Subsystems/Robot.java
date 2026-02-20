package org.firstinspires.ftc.teamcode.robotControl.Subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis.ChassisLocal;

public class Robot {

    public final ChassisLocal chassis;
    public final Turret turret;
    public final Intake intake;
    public final TransferGate gate;
    public final Vision vision;

    public Robot(HardwareMap hardwareMap, Pose startingPose) {

        chassis = new ChassisLocal(hardwareMap, startingPose);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new TransferGate(hardwareMap);
        vision = new Vision(hardwareMap);

    }
}