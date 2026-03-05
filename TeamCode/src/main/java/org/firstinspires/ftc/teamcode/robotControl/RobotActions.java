package org.firstinspires.ftc.teamcode.robotControl;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis.ChassisLocal;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.LLResult;
import java.util.List;

public class RobotActions implements BlueUniversalConstants, org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TurretConstants {

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

    public void launch(double speed, boolean bumperPressed, Pose targ) {
        boolean justPressed = bumperPressed && !lastBumper;
        boolean justReleased = !bumperPressed && lastBumper;
        lastBumper = bumperPressed;
        if (justPressed) {
            launchStartTime = System.currentTimeMillis();
            shotCount = 0;
        }
        if (bumperPressed) {
            gate.open();
            turret.setFeedPower(speed);
            // Check if we're in far zone
            double dist = chassisLocal.getDistance(targ);
            int zone = VelocityLookupTable.getZone(dist);
            if (zone == 3) {
                intake.simpleIntake(-speed * 0.7);
                farShootingActive = true;
                // How many balls have been fed based on elapsed time
                long elapsed = System.currentTimeMillis() - launchStartTime;
                int expectedBalls = (int) (elapsed / 50);
                if (expectedBalls > shotCount) {
                    shotCount = expectedBalls;
                }
                // Shift hood down per ball, clamped to mid position max
                double hoodPos = FAR_HOOD_POSITION + (shotCount * 0.3);
                hoodPos = Math.min(hoodPos, MID_HOOD_POSITION);
                turret.setHoodPos(hoodPos);
            } else {
                turret.setFeedPower(speed);
                intake.simpleIntake(-speed);
                farShootingActive = false;
            }
        }
        if (justReleased || !bumperPressed) {
            gate.block();
            turret.setFeedPower(0);
            farShootingActive = false;
            shotCount = 0;
        }
    }

    // To be tested
    public void relocalizeBlue(Pose target, Telemetry telemetry) {
        if (!vision.hasTarget())
            return;

        double tx = vision.getTx();
        double dist = vision.getDistance();
        telemetry.addData("Dist", dist);

        // Calculate robot position relative to target using tx and distance
        double angleRad = Math.toRadians(tx);
        double dx = dist * Math.sin(angleRad);
        double dy = dist * Math.cos(angleRad);

        double newX = target.getX() + dx;
        double newY = target.getY() - dy;

        Pose newPose = new Pose(newX, newY, chassisLocal.getPose().getHeading());

        telemetry.addData("New Position", newPose);

        // maybe add heading?
        chassisLocal.setPose(newPose);
    }

    public void relocalizeRed(Pose target) {
        if (!vision.hasTarget())
            return;

        double tx = vision.getTx();
        double dist = vision.getDistance();

        // Calculate robot position relative to target using tx and distance
        double angleRad = Math.toRadians(tx);
        double dx = dist * Math.sin(angleRad);
        double dy = dist * Math.cos(angleRad);

        double newX = target.getX() - dx;
        double newY = target.getY() - dy;

        // maybe add heading?
        chassisLocal.setPose(new Pose(newX, newY, chassisLocal.getPose().getHeading()));
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

    public void LimelightRelocal(Telemetry telemetry) {
        Pose llPose = vision.getPoseLimelight();
        if (llPose == null) {
            telemetry.addData("Limelight relocal", "No valid pose (no target or limelight)");
            return;
        }
        chassisLocal.setPose(llPose);
        telemetry.addData("Limelight relocal", "OK (%.1f, %.1f, %.0f°)",
                llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
    }

    // we have a control for intake in this class in case intaking becomes more
    // complex
    public void intake(double power) {
        intake.simpleIntake(power);
        if (power > 0) {
            turret.setFeedPower(-power);
        }
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

    public void autoAdjustVelo(Pose targ) {
        // If launch() is actively controlling the hood for far shots, skip
        if (farShootingActive)
            return;
        double dist = chassisLocal.getDistance(targ);
        int zone = ServoLookupTable.getZone(dist);
        if (zone == 1) {
            turret.setVelocity(CLOSE_VELO);
        } else if (zone == 2) {
            turret.setVelocity(MID_VELO);
        } else {
            turret.setVelocity(FAR_VELO);
        }
    }

    public void autoVelocity(Pose targ) {
        double dist = chassisLocal.getDistance(targ);
        double autoVelocity = VelocityLookupTable.getVelocity(dist);
        turret.setVelocity(autoVelocity);
    }

    public void adjustShootingParams(Pose targ) {
        autoVelocity(targ);
        autoAdjustHood(targ);
    }

    public void adjustShootingParamsTest(Pose targ) {
        autoAdjustVelo(targ);
    }

    public void aimRotatorLocal(Pose targ, @NonNull Telemetry telemetry) {
        double currentHeading = chassisLocal.getPose().getHeading();
        long currentTime = System.currentTimeMillis();

        if (lastAimTime != 0) {
            double dtSeconds = (currentTime - lastAimTime) / 1000.0;
            if (dtSeconds > 0) {
                double headingDiff = currentHeading - lastHeadingForAim;
                while (headingDiff > Math.PI)
                    headingDiff -= 2 * Math.PI;
                while (headingDiff < -Math.PI)
                    headingDiff += 2 * Math.PI;

                double angularVelocity = Math.abs(headingDiff) / dtSeconds;

                if (angularVelocity > FREEZE_THRESHOLD_RAD_PER_SEC) {
                    isTurretFrozen = true;
                } else if (angularVelocity < UNFREEZE_THRESHOLD_RAD_PER_SEC) {
                    isTurretFrozen = false;
                }
            }
        }

        lastHeadingForAim = currentHeading;
        lastAimTime = currentTime;

        if (!isTurretFrozen) {
            double angle = chassisLocal.calculateTurretAngle(targ);
            telemetry.addData("Angle with localization", angle);
            turret.setRotatorToAngle(angle);
            telemetry.addData("Turret Status", "Aiming");
        } else {
            telemetry.addData("Turret Status", "Frozen (Turning too fast)");
            telemetry.addData("Angle with localization", "Frozen");
        }
    }

    public void aimRotatorLocalOld(Pose targ, @NonNull Telemetry telemetry) {
        double currentHeading = chassisLocal.getPose().getHeading();
        long currentTime = System.currentTimeMillis();

        if (lastAimTime != 0) {
            double dtSeconds = (currentTime - lastAimTime) / 1000.0;
            if (dtSeconds > 0) {
                double headingDiff = currentHeading - lastHeadingForAim;
                while (headingDiff > Math.PI)
                    headingDiff -= 2 * Math.PI;
                while (headingDiff < -Math.PI)
                    headingDiff += 2 * Math.PI;

                double angularVelocity = Math.abs(headingDiff) / dtSeconds;

                if (angularVelocity > FREEZE_THRESHOLD_RAD_PER_SEC) {
                    isTurretFrozen = true;
                } else if (angularVelocity < UNFREEZE_THRESHOLD_RAD_PER_SEC) {
                    isTurretFrozen = false;
                }
            }
        }

        lastHeadingForAim = currentHeading;
        lastAimTime = currentTime;

        if (!isTurretFrozen) {
            double angle = chassisLocal.getTurretAngle(targ);
            telemetry.addData("Angle with localization", angle);
            turret.setRotatorToAngle(angle);
            telemetry.addData("Turret Status", "Aiming");
        } else {
            telemetry.addData("Turret Status", "Frozen (Turning too fast)");
            telemetry.addData("Angle with localization", "Frozen");
        }
    }

    public void aim() {

    }

    // public void relocalize() {
    // Pose pose = vision.getEstimatedPose();
    // drive.setPoseEstimate(pose);
    // }
    //
    public void updateLimelight() {
        if (vision != null) {
            // MT2 logic requires the chassis heading
            vision.updateRobotOrientation(Math.toDegrees(chassisLocal.getPose().getHeading()));
        }
    }

    /**
     * Performs a full heading + position snap using the "Vision Yaw - Turret Angle" formula.
     */
    public boolean relocalizeFull(Telemetry telemetry) {
        LLResult result = vision.getLatestResult();
        if (result == null || !result.isValid()) return false;

        List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return false;

        Pose3D botposeMT1 = result.getBotpose();
        if (botposeMT1 == null) return false;

        // Position Conversion
        org.firstinspires.ftc.robotcore.external.navigation.Position posInches = botposeMT1.getPosition().toUnit(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double invertedLLX = botposeMT1.getPosition().y * METERS_TO_INCHES; 
        double invertedLLY = -botposeMT1.getPosition().x * METERS_TO_INCHES;

        double pedroX = invertedLLX + FIELD_OFFSET_X;
        double pedroY = invertedLLY + FIELD_OFFSET_Y;

        // Heading Calculation
        double mt1Yaw = botposeMT1.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        double turretAngleDeg = (turret.getRotatorPos() - ROTATOR_ZERO_TICKS) / TICKS_PER_DEGREE; 
        double calculatedChassisHeadingDeg = mt1Yaw - turretAngleDeg;

        while (calculatedChassisHeadingDeg > 180) calculatedChassisHeadingDeg -= 360;
        while (calculatedChassisHeadingDeg < -180) calculatedChassisHeadingDeg += 360;

        double headingRad = Math.toRadians(calculatedChassisHeadingDeg) - Math.PI / 2;
        chassisLocal.setPose(new Pose(pedroX, pedroY, headingRad));
        return true;
    }

    /**
     * Resets only the (x, y) position based on AprilTag data, maintaining current heading.
     */
    public boolean relocalizePositionOnly() {
        LLResult result = vision.getLatestResult();
        if (result == null || !result.isValid()) return false;

        Pose3D botposeMT1 = result.getBotpose();
        if (botposeMT1 == null) return false;

        double invertedLLX = botposeMT1.getPosition().y * METERS_TO_INCHES;
        double invertedLLY = -botposeMT1.getPosition().x * METERS_TO_INCHES;

        double pedroX = invertedLLX + FIELD_OFFSET_X;
        double pedroY = invertedLLY + FIELD_OFFSET_Y;

        chassisLocal.setPose(new Pose(pedroX, pedroY, chassisLocal.getPose().getHeading()));
        return true;
    }
}
