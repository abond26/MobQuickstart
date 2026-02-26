package org.firstinspires.ftc.teamcode.robotControl;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis.ChassisLocal;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;

public class RobotActions implements BlueUniversalConstants{

    private ChassisLocal chassisLocal;
    private Turret turret;
    private Intake intake;
    private TransferGate gate;
    private Vision vision;

    public RobotActions(ChassisLocal chassis, Vision vision, Turret turret, TransferGate gate, Intake intake) {
        this.chassisLocal = chassis;
        this.turret = turret;
        this.intake = intake;
        this.gate = gate;
        this.vision = vision;

    }

    public void launch(double speed, boolean bumperPressed){
        if (bumperPressed) {
            gate.open();
            turret.setFeedPower(speed);
            intake.simpleIntake(-speed);
        }
        else{
            gate.block();
            turret.setFeedPower(0);
        }
    }

    //Manual hood Control
    public void hoodControl(boolean xPressed, boolean bPressed){
        if (xPressed){
            turret.shiftHood(-hoodIncrement);
        }
        if (bPressed){
            turret.shiftHood(hoodIncrement);
        }
    }


    //we have a control for intake in this class in case intaking becomes more complex
    public void intake(double power){
        intake.simpleIntake(power);
        if (power > 0){
            turret.setFeedPower(-power);
        }
    }

    public void autoAdjustHood(Pose targ){
        double dist = chassisLocal.getDistance(targ);;
        int zone = VelocityLookupTable.getZone(dist);
        if (zone == 1) {
            // Zone 1: Close range (< 140)
            turret.setHoodPos(CLOSE_HOOD_POSITION);
        } else if (zone == 2) {
            // Zone 2: Mid range (140-200)
            turret.setHoodPos(MID_HOOD_POSITION);
        } else {
            // Zone 3: Far range (>= 200)
            turret.setHoodPos(FAR_HOOD_POSITION);
        }

    }

    public void autoVelocity(Pose targ){
        double dist = chassisLocal.getDistance(targ);
        double autoVelocity = VelocityLookupTable.getVelocity(dist);
        turret.setVelocity(autoVelocity);
    }

    public void adjustShootingParams(Pose targ) {
        autoVelocity(targ);
        autoAdjustHood(targ);
    }

    public void aimRotatorLocal(Pose targ, @NonNull Telemetry telemetry){
        double angle = chassisLocal.calculateTurretAngle(targ);
        telemetry.addData("Angle with localization", angle);
        turret.setRotatorToAngle(angle);
    }

    public void aimRotatorLocalOld(Pose targ, @NonNull Telemetry telemetry){
        double angle = chassisLocal.getTurretAngle(targ);
        telemetry.addData("Angle with localization", angle);
        turret.setRotatorToAngle(angle);
    }

    public void aim(){

    }

//    public void relocalize() {
//        Pose pose = vision.getEstimatedPose();
//        drive.setPoseEstimate(pose);
//    }
//
//    public void autoAdjustShooter() {
//        double distance = vision.getTargetDistance();
//        shooter.setVelocityFromDistance(distance);
//    }
}
