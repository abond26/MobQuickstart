package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.robotControl.ShotTimeLookupTable;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
public class ChassisLocal implements DriveConstants{

    private Follower follower;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotor rotator;


    //we don't put this in the DriveConstants class because it changes (i.e. it isn't a static final)
    private double lastTurretAngleDeg = 0;

    public ChassisLocal(HardwareMap hardwareMap, Pose startPose) {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

    }

    public void startTeleop() {
        follower.startTeleopDrive();
    }

    public void update() {
        follower.update();
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }
    public Vector getVelocity() {return follower.getVelocity();}

    public void drive(double y, double x, double r) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        leftFront.setPower((y + x + r) / denominator);
        leftRear.setPower((y - x + r) / denominator * driveMultiplier);
        rightFront.setPower((y - x - r) / denominator);
        rightRear.setPower((y + x - r) / denominator * driveMultiplier);
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

        return sillyTarget;
    }


    public double getDistance(Pose target){
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


    //----------------------------------------------------------------
    //TO CHANGE? The below code is chat code
    public double calculateTurretAngle(Pose target) {
        Pose robotPose = getPose();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngle = angleToGoal - Math.toDegrees(robotPose.getHeading());
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        return -turretAngle;
    }

    //----------------------------------------------------------------
    //TO CHANGE? -> Put in Turret class
//    public void setTurretAngle(double turretAngleDeg) {
//
//        double fracOf180 = Math.toRadians(turretAngleDeg) / Math.PI;
//        int targetTicks = (int)(fracOf180 * motor180Range);
//
//        rotator.setTargetPosition(targetTicks);
//    }
}