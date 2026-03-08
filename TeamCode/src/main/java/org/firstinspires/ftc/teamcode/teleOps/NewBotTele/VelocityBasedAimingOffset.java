package org.firstinspires.ftc.teamcode.teleOps.NewBotTele;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

/**
 * Velocity-based aiming offset for shooting while moving.
 *
 * Uses the approach from Team 23435 Gyrobotic Droids "Hood Angle and Velocity Calcs":
 * - Decompose robot velocity into radial (along line to goal) and tangential (perpendicular).
 * - Turret offset (view from above) = arctan(Vrt / Vx,compensated).
 * - Flight time is computed from distance and launch velocity (no separate constant to tune).
 *
 * <p>Tuning variables for shooting while moving (only these):
 * <ul>
 *   <li>VELOCITY_THRESHOLD – speed above which lead is applied (in/s)</li>
 *   <li>MAX_OFFSET_DEGREES – clamp on turret offset (degrees)</li>
 *   <li>OFFSET_SCALING_FACTOR – scale factor if consistently over/under leading</li>
 * </ul>
 * Ball speed (in/s) is supplied by the opmode, e.g. from launcher ticks/s → RPM → π×diameter (in/s).
 */
public class VelocityBasedAimingOffset {

    private final Follower follower;

    /** Velocity below which no offset is applied (in/s). */
    private static final double VELOCITY_THRESHOLD = 5.0;

    /** Fallback time (s) only when 3-arg overload is used; prefer 5-arg with distance + ball speed. */
    private static final double PROJECTILE_FLIGHT_TIME_FALLBACK = 0.3;

    /** Clamp turret offset to ± this (degrees). Set high (e.g. 90) so full lead is used; reduce if turret range or stability needs it. */
    private static final double MAX_OFFSET_DEGREES = 90.0;

    /** Scale factor for the computed offset (tune if consistently over/under leading). */
    private static final double OFFSET_SCALING_FACTOR = 1.0;

    public VelocityBasedAimingOffset(Follower follower) {
        this.follower = follower;
    }

    /**
     * Adjusted aiming angle with velocity compensation. Use when you have distance and ball speed (in/s).
     * Flight time = distance / ballSpeedInchesPerSec (no ticks constant here; opmode converts setVelocity(ticks/s) → in/s).
     */
    public double calculateAdjustedAngle(double baseAngleDeg, Pose robotPose, Pose targetPose,
                                         double distanceToTarget, double ballSpeedInchesPerSec) {
        double timeToGoal = (ballSpeedInchesPerSec > 0 && distanceToTarget > 0)
                ? distanceToTarget / ballSpeedInchesPerSec
                : PROJECTILE_FLIGHT_TIME_FALLBACK;
        return calculateAdjustedAngle(baseAngleDeg, robotPose, targetPose, timeToGoal);
    }

    /** Legacy: no distance/velocity → uses fallback constant time. */
    public double calculateAdjustedAngle(double baseAngleDeg, Pose robotPose, Pose targetPose) {
        return calculateAdjustedAngle(baseAngleDeg, robotPose, targetPose, PROJECTILE_FLIGHT_TIME_FALLBACK);
    }

    private double calculateAdjustedAngle(double baseAngleDeg, Pose robotPose, Pose targetPose, double timeToGoal) {
        Vector velocity = follower.getVelocity();
        double globalVelX = velocity.dot(new Vector(1.0, 0));
        double globalVelY = velocity.dot(new Vector(1.0, Math.PI / 2));
        double velocityMagnitude = Math.sqrt(globalVelX * globalVelX + globalVelY * globalVelY);

        if (velocityMagnitude < VELOCITY_THRESHOLD) {
            return baseAngleDeg;
        }

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        if (distanceToTarget < 1.0) {
            return baseAngleDeg;
        }

        double thetaLine = Math.atan2(dy, dx);
        double thetaVelocity = Math.atan2(globalVelY, globalVelX);
        double theta = thetaVelocity - thetaLine;

        double Vrr = velocityMagnitude * Math.cos(theta);
        double Vrt = velocityMagnitude * Math.sin(theta);

        double horizontalBallSpeed = distanceToTarget / timeToGoal;
        double VxCompensated = horizontalBallSpeed + Vrr;

        // Lead = aim so ball (muzzle + robot velocity) goes at goal. Robot velocity pushes ball → aim the other way.
        // Offset = arctan(Vrt/VxCompensated): e.g. moving backward (Vrt>0) pushes ball back → aim forward/right (positive offset).
        double offsetRadians = Math.atan2(Vrt, VxCompensated);
        double offsetDegrees = Math.toDegrees(offsetRadians) * OFFSET_SCALING_FACTOR;

        if (offsetDegrees > MAX_OFFSET_DEGREES) {
            offsetDegrees = MAX_OFFSET_DEGREES;
        } else if (offsetDegrees < -MAX_OFFSET_DEGREES) {
            offsetDegrees = -MAX_OFFSET_DEGREES;
        }

        double adjustedAngle = baseAngleDeg + offsetDegrees;
        while (adjustedAngle > 180) adjustedAngle -= 360;
        while (adjustedAngle < -180) adjustedAngle += 360;

        return adjustedAngle;
    }
    
    /**
     * Gets the current velocity magnitude for telemetry/debugging.
     * 
     * @return Current velocity magnitude in inches per second
     */
    public double getCurrentVelocity() {
        Vector velocity = follower.getVelocity();
        double globalVelX = velocity.dot(new Vector(1.0, 0));
        double globalVelY = velocity.dot(new Vector(1.0, Math.PI / 2));
        return Math.sqrt(globalVelX * globalVelX + globalVelY * globalVelY);
    }
    
    /**
     * Checks if the robot is moving fast enough to require offset.
     * 
     * @return true if velocity is above threshold
     */
    public boolean isMoving() {
        return getCurrentVelocity() >= VELOCITY_THRESHOLD;
    }
    
}
