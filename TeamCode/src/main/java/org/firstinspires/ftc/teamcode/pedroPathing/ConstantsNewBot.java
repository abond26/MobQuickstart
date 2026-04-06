package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .centripetalScaling(0)
            .headingPIDFCoefficients(new PIDFCoefficients(0.846, 0.01, 0.095, 0.04))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.25, 0.093269, 0.001974))
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.22, 0, 0.015, 0.0085))
//            .drivePIDFCoefficients((new FilteredPIDFCoefficients(0.0075, 0.001, 0.00003, 0.03, 0.0095)))
            .forwardZeroPowerAcceleration(-73.2273364)
            .lateralZeroPowerAcceleration(-89.668392299)
//           .useSecondaryTranslationalPIDF(false)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0.0001,0.06,0))
//            .useSecondaryHeadingPIDF(false)
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.2,0.0001,0.06,0))
//           .useSecondaryDrivePIDF(false)
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.75,0.000075,0.03,0.6,0.01))
            .mass(9.979);



    public static PathConstraints pathConstraints = new PathConstraints(100,
            100,
            1,
            1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(87.113686869)
            .yVelocity(68.083442748)





            .maxPower(1)
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
            .forwardPodY(3)
            .strafePodX(2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
