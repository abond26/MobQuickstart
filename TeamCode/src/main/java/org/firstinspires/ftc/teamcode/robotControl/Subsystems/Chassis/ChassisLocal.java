package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis;

import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;
import com.pedropathing.paths.PathChain;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.ShotTimeLookupTable;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
public class ChassisLocal implements DriveConstants{

    private Follower follower;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;


    //we don't put this in the DriveConstants class because it changes (i.e. it isn't a static final)
    private double lastTurretAngleDeg = 0;

    public ChassisLocal(HardwareMap hardwareMap, Pose startPose) {

        follower = ConstantsNewBot.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        follower.update();


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void startTeleop() {
        follower.startTeleopDrive();
    }

    public Follower getFollower() {
        return follower;
    }

    private double lastHeading = 0;
    private long lastTime = 0;
    private double headingVelocityRad = 0;

    public void update() {
        follower.update();
        long currentTime = System.nanoTime();
        double currentHeading = follower.getPose().getHeading();
        if (lastTime != 0) {
            double dt = (currentTime - lastTime) / 1e9;
            if (dt > 0 && dt < 0.5) { // Prevent massive spikes
                double dH = currentHeading - lastHeading;
                while (dH > Math.PI) dH -= 2 * Math.PI;
                while (dH < -Math.PI) dH += 2 * Math.PI;
                headingVelocityRad = dH / dt;
            }
        }
        lastHeading = currentHeading;
        lastTime = currentTime;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }
    public Vector getVelocity() {return follower.getVelocity();}

    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }

    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }


    public void drive(double y, double x, double r) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        leftFront.setPower((y + x + r) / denominator);
        leftRear.setPower((y - x + r) / denominator );
        rightFront.setPower((y - x - r) / denominator);
        rightRear.setPower((y + x - r) / denominator);
    }

    public Pose sillyTargetPose(Pose target){
        double dist = getDistance(target);

        Vector velocity = getVelocity();
        double vx = velocity.getMagnitude() * Math.cos(velocity.getTheta());
        double vy = velocity.getMagnitude() * Math.sin(velocity.getTheta());
        Pose sillyTarget = new Pose(
                target.getX() - vx * ShotTimeLookupTable.getTime(dist),
                target.getY() - vy * ShotTimeLookupTable.getTime(dist)
        );
        if (dist < 120){
            return sillyTarget;
        }else{
            return target;
        }
    }


    public double getDistance(@NonNull Pose target){
        double currentX = getPose().getX();
        double currentY = getPose().getY();
        double distance = Math.sqrt(Math.pow(currentX - target.getX(), 2) + Math.pow(currentY - target.getY(), 2));
        return distance;
    }


    //Name changed from "alignTurret"
    public double getTurretAngle(@NonNull Pose target) {
        Pose robotPose = getPose();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngle = angleToGoal - Math.toDegrees(robotPose.getHeading());

        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        double normalized = turretAngle;
        double diff = turretAngle - lastTurretAngleDeg;
        if (diff > 90) turretAngle -= 360;
        else if (diff < -90) turretAngle += 360;
        // Reset at 360° so we don't accumulate to 720°; command 0 and keep angle in [-180,180]
        if (turretAngle >= 180 || turretAngle <= -180) {
            lastTurretAngleDeg = normalized;
            return 0;
        }
        lastTurretAngleDeg = turretAngle;
        return -turretAngle;
    }


    //kishens great code//
    public double calculateTurretAngle(@NonNull Pose target) {
        Pose robotPose = getPose();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        
        double predictedHeading = Math.toDegrees(robotPose.getHeading()) + (Math.toDegrees(headingVelocityRad) * 0.15);
        
        double turretAngle = angleToGoal - predictedHeading;
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        return -turretAngle;
    }
//
    /*if blue:
         y = -x + 144
         if red:
         y = x
         */
    public Pose getTargetPoseBlueSimple(@NonNull Pose goalPose){
        Pose bestPose;
        Pose currentPosition = getPose();
        double y = -currentPosition.getX() + 144;
        if (currentPosition.getY() < y) {
            bestPose = new Pose(4, 144, 0);
        }
        else {
            bestPose = new Pose(0, 140, 0);
        }
        return bestPose;
    }

    public Pose getTargetPoseBlueLinear(@NonNull Pose goalPose){
        Pose bestPose;
        Pose currentPosition = getPose();
        double y = -currentPosition.getX() + 144;
        double dy = Math.abs(y - currentPosition.getY());
        double distanceToLine = dy * Math.cos(Math.toRadians(45));
        double distanceToGoal = getDistance(goalPose);

        //angle between imaginary line and goal
        double angle = Math.acos(distanceToGoal/distanceToLine);
        /*For easy scaling, the extreme has to be the largest number
        - thus subtract 90 and get the magnitude of that.
        Then divide by 45 to get it to be a scale between 0 and 1.
         */
        double scaledAngle = Math.abs(angle - 90)/45;

        //Linear adjustment: Max adjust = m and scaledAngle = x
        double adjustment = MAX_GOAL_ADJUSTMENT * scaledAngle;
        if (currentPosition.getY() < y) {
            bestPose = new Pose(adjustment, 144, 0);
        }
        else{
            bestPose = new Pose(0, 144 - adjustment, 0);
        }

        return bestPose;
    }
    //

    public boolean isShootingPosition(){
        boolean inPosition = false;
        Pose currentPosition = getPose();
        double y = currentPosition.getY();
        double x = currentPosition.getX();
        double leftUpBound = -x + 144 - boundaryExpansion;
        double leftBottomBound = x - 48 + boundaryExpansion;
        double rightUpBound = x - boundaryExpansion;
        double rightBottomBound = -x + 144 - 48 + boundaryExpansion;

        if (currentPosition.getX() < 72) {
            if (y > leftUpBound || y < leftBottomBound) {
                inPosition = true;
            }
        }
        else{
            if (y > rightUpBound || y < rightBottomBound) {
                inPosition = true;
            }
        }
        return inPosition;




        //Left Side
        //y = -x + 144 - boundaryExpansion;
        //y = x - 48

        //right side
        //y = x
        //y = -x + 144

    }
    // Inside ChassisLocal.java
    /**
     * @param velocityScale multiply reported chassis velocity before applying lead (use above 1.0 in
     *                    auton when path follower velocity is smaller than teleop stick drive).
     */
    public Pose getLeadTargetPose(Pose target, double velocityScale) {
        double dist = getDistance(target);
        Vector velocity = getVelocity();

        double g = 386.088;
        double scoreHeight = 50;
        double scoreAngle = Math.toRadians(-60);

        double time = Math.sqrt(2 * (scoreHeight - dist * Math.tan(scoreAngle)) / g);

        double mag = velocity.getMagnitude() * velocityScale;
        double vx = mag * Math.cos(velocity.getTheta());
        double vy = mag * Math.sin(velocity.getTheta());

        return new Pose(
                target.getX() - vx * time,
                target.getY() - vy * time,
                target.getHeading()
        );
    }

    public Pose getLeadTargetPose(Pose target) {
        return getLeadTargetPose(target, 1.0);
    }
    public double getVeloX(){
        Vector velocity = getVelocity();
        return  velocity.getXComponent();
    }
    public double getVeloY(){
        Vector velocity = getVelocity();
        return  velocity.getYComponent();
    }



    //----------------------------------------------------------------
    //TO CHANGE? -> Put in Turret class
//    public void setTurretAngle(double turretAngleDeg) {
//
//        double fracOf180 = Math.toRadians(turretAngleDeg) / Math.PI;
//        double targetTicks = (int)(fracOf180 * motor180Range);
//
//        rotator.setPosition(targetTicks);
//    }
    //
}