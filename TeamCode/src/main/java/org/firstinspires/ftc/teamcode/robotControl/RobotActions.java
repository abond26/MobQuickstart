package org.firstinspires.ftc.teamcode.robotControl;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis.ChassisLocal;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.VelocityLookupTable;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TestTurret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;

public class RobotActions implements BlueUniversalConstants, TurretConstants {

    private ChassisLocal chassisLocal;
    private Turret turret;
    private Intake intake;
    private TransferGate gate;
    private Vision vision;

    /**
     * When non-null, used as shooting target instead of
     * {@link BlueUniversalConstants#target}.
     */
    private Pose shootingTargetOverride = null;
    private boolean lastBumper = false;
    private long launchStartTime = 0;
    private int shotCount = 0;
    private boolean farShootingActive = false;

    private double lastHeadingForAim = 0;
    private long lastAimTime = 0;
    private boolean isTurretFrozen = false;
    private final double FREEZE_THRESHOLD_RAD_PER_SEC = 1.5;
    private final double UNFREEZE_THRESHOLD_RAD_PER_SEC = 0.5;

    public RobotActions(ChassisLocal chassis, Vision vision, Turret turret, TransferGate gate, Intake intake) {
        this.chassisLocal = chassis;
        this.turret = turret;
        this.intake = intake;
        this.gate = gate;
        this.vision = vision;

    }

    public void launch(double speed, boolean bumperPressed) {
        if (bumperPressed) {
            gate.open();
            intake.simpleIntake(-speed);
        } else {
            gate.block();
        }
    }

    // Manual hood Control
    public void hoodControl(boolean xPressed, boolean bPressed) {
        if (xPressed) {
            turret.shiftHood(-hoodIncrement);
        }
        if (bPressed) {
            turret.shiftHood(hoodIncrement);
        }
    }

    public void setRobotPose(Pose pose) {
        chassisLocal.setPose(pose);
    }

    public void setTargetPose(Pose targetPose) {
        this.shootingTargetOverride = targetPose;
    }

    /**
     * Returns the current shooting target (override if set, otherwise default from
     * constants).
     */
    public Pose getShootingTarget() {
        return shootingTargetOverride != null ? shootingTargetOverride : target;
    }

    // we have a control for intake in this class in case intaking becomes more
    // complex
    public void intake(double power) {
        intake.simpleIntake(power);
    }

    public void autoAdjustHood(Pose targ) {
        // If launch() is actively controlling the hood for far shots, skip
        if (farShootingActive)
            return;
        double dist = chassisLocal.getDistance(targ);
        int zone = VelocityLookupTable.getZone(dist);
        if (zone == 1) {
            turret.setHoodPos(CLOSE_HOOD_POSITION);
        } else if (zone == 2) {
            turret.setHoodPos(MID_HOOD_POSITION);
        } else {
            turret.setHoodPos(FAR_HOOD_POSITION);
        }
    }

    // public void autoAdjustVelo(Pose targ) {
    // // If launch() is actively controlling the hood for far shots, skip
    // if (farShootingActive)
    // return;
    // double dist = chassisLocal.getDistance(targ);
    // int zone = ServoLookupTable.getZone(dist);
    // if (zone == 1) {
    // turret.setVelocity(CLOSE_VELO);
    // } else if (zone == 2) {
    // turret.setVelocity(MID_VELO);
    // } else {
    // turret.setVelocity(FAR_VELO);
    // }
    // }

    public void autoVelocity(Pose targ) {
        double dist = chassisLocal.getDistance(targ);
        double autoVelocity = VelocityLookupTable.getVelocity(dist);
        turret.setVelocity(autoVelocity);
    }

    public void autoVelocityEquation(Pose targ) {
        double dist = chassisLocal.getDistance(targ);
        double robotY = chassisLocal.getPose().getY();
        double targetRPM;

        if (robotY < 30) {
           return;
        } else {


            targetRPM = 0.012264577 * Math.pow(dist, 2) + 5.4005132 * dist + 1470;
        }

        if (turret instanceof TestTurret) {
            ((TestTurret) turret).setTargetRPM(targetRPM);
        } else {
            double ticksPerSec = (targetRPM * ENCODER_CPM * GEAR_RATIO) / 60.0;
            turret.setVelocity(ticksPerSec);
        }
    }

    public void adjustShootingParams(Pose targ) {
        autoVelocity(targ);
        autoAdjustHood(targ);
    }

    public void aimRotatorLocal(Pose targ, @NonNull Telemetry telemetry) {
        double angle = chassisLocal.calculateTurretAngle(targ);
        telemetry.addData("Angle with localization", angle);
        turret.setRotatorToAngle(angle);
    }


    public void targetFixX(double amount) {
        Pose current = getShootingTarget();
        shootingTargetOverride = new Pose(current.getX() + amount, current.getY(), current.getHeading());
    }
    public void ChangeTargBlue() {
        double robotY = chassisLocal.getPose().getY();

        if (robotY < 30) {
            shootingTargetOverride = new Pose(7, 144, 144);
        } else if (robotY >= 30 && robotY < 109.851150202977) {
            shootingTargetOverride = new Pose(-1, 144, 0);
        } else {
            shootingTargetOverride = new Pose(7, 140, 0);
        }
    }
    public void ChangeTargRed() {
        double robotY = chassisLocal.getPose().getY();

        if (robotY < 30) {
            shootingTargetOverride = new Pose(72, 72, 0);
        } else if (robotY >= 30 && robotY < 60) {
            shootingTargetOverride = new Pose(72, 100, 0);
        } else {
            shootingTargetOverride = new Pose(72, 140, 0);
        }
    }


    // public void autoAdjustShooter() {
    // double distance = vision.getTargetDistance();
    // shooter.setVelocityFromDistance(distance);
    // }
}