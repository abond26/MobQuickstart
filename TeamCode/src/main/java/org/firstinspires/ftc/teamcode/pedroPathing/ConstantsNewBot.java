package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ConstantsNewBot {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .centripetalScaling(0.00035)
            .headingPIDFCoefficients(new PIDFCoefficients(1.08, 0.0001, 0.032, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.18, 0.00001, 0.0134, 0))
            .drivePIDFCoefficients((new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.01, 0.01)))
           .forwardZeroPowerAcceleration(-44.4019822807)
            .lateralZeroPowerAcceleration(-60.34057178113703)
            .mass(12.06556);



    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.2,
            1);;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(79.4311860736)
            .yVelocity(63.8537607268056)





//            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    //left front changed left rear
    //left rear changed right rear
    //right front changed right front
    //right rear
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.25)
            .strafePodX(-7.875)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
