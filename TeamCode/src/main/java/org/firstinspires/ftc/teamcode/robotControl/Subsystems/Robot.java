package org.firstinspires.ftc.teamcode.robotControl.Subsystems;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis.ChassisLocal;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;

public class Robot {

    public final ChassisLocal chassisLocal;
    public final Turret turret;
    public final Intake intake;
    public final TransferGate gate;
    public final Vision vision;
//
    public Robot(HardwareMap hardwareMap, Pose startingPose, int pipeline) {


        chassisLocal = new ChassisLocal(hardwareMap, startingPose);
        Log.w("Chassis", "Chassis Loaded");
        //
        turret = new Turret(hardwareMap);
        Log.w("Turret", "Turret Loaded");

        intake = new Intake(hardwareMap);
        Log.w("Intake", "Intake Loaded");
        //

        gate = new TransferGate(hardwareMap);
        Log.w("TransferGate", "TransferGate Loaded");

        vision = new Vision(hardwareMap, pipeline);
        Log.w("Vision", "Vision Loaded");


    }
}