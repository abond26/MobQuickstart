package org.firstinspires.ftc.teamcode.robotControl.Subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis.ChassisLocal;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Relocalization.Relocalization;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;

public class Robot {

    public final ChassisLocal chassisLocal;
    public final Turret turret;
    public final Intake intake;
    public final TransferGate gate;
    public final Vision vision;
    //public final Relocalization relocalization;

    public Robot(HardwareMap hardwareMap, Pose startingPose, int pipeline) {

        chassisLocal = new ChassisLocal(hardwareMap, startingPose);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new TransferGate(hardwareMap);
        vision = new Vision(hardwareMap, pipeline);
        //relocalization = new Relocalization(hardwareMap, chassisLocal);

    }
}