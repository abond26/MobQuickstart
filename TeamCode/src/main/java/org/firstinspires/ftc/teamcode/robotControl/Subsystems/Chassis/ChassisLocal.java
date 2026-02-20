package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
public class ChassisLocal {

    private Follower follower;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private DcMotor rotator;

    private double lastTurretAngleDeg = 0;
    private int motor180Range = 1250;

    public ChassisLocal(HardwareMap hardwareMap, Pose startPose) {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);
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

    public void drive(double y, double x, double r) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        leftFront.setPower((y + x + r) / denominator);
        leftRear.setPower((y - x + r) / denominator);
        rightFront.setPower((y - x - r) / denominator);
        rightRear.setPower((y + x - r) / denominator);
    }
    //----------------------------------------------------------------
    //TO CHANGE?
    public double calculateTurretAngle(Pose target) {

        Pose robotPose = getPose();

        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();

        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg = Math.toDegrees(robotPose.getHeading());

        double turretAngle = -(angleToGoal - headingDeg);

        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        double diff = turretAngle - lastTurretAngleDeg;
        if (diff > 90) turretAngle -= 360;
        if (diff < -90) turretAngle += 360;

        lastTurretAngleDeg = turretAngle;
        return turretAngle;
    }

    //----------------------------------------------------------------
    //TO CHANGE?
    public void setTurretAngle(double turretAngleDeg) {

        double fracOf180 = Math.toRadians(turretAngleDeg) / Math.PI;
        int targetTicks = (int)(fracOf180 * motor180Range);

        rotator.setTargetPosition(targetTicks);
    }
}