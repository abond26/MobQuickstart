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

    //Hood Control
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
            turret.setFeedPower(power);
        }
    }

    public void aimRotatorLocal(Pose target, @NonNull Telemetry telemetry){
        double angle = chassisLocal.calculateTurretAngle(target);
        telemetry.addData("Angle with localization", angle);
        turret.setRotatorToAngle(angle);
    }

    public void aimRotatorLocalOld(Pose target, @NonNull Telemetry telemetry){
        double angle = chassisLocal.getTurretAngle(target);
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
