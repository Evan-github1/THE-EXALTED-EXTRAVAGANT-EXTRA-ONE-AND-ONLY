package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.28)
            .forwardZeroPowerAcceleration(-25.3459)
            .lateralZeroPowerAcceleration(-64.70768)
            // TODO: Tune translational PID to reduce endpoint overshoot.
            //  Try increasing D from 0.02 toward 0.03-0.05. On problem paths, also try
            //  path.setTimeoutConstraint(ms) to give the follower more correction time.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3,0,0.02,0.018))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2,0,0.05,0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0.001,0.00002,0.5,0.01))
            .centripetalScaling(0.0005)
            ;

    // TODO: 4-param PathConstraints is (tValue, timeout, brakingStrength, brakingStart).
    //  Defaults for omitted params: velocity=0.1, translational=0.1, heading=0.007, bezierSearchLimit=10.
    //  Current values (0.995, 100, 1, 1) use defaults for settling constraints — tune timeout or
    //  use the full 8-param constructor if finer control is needed. Per-path: path.setTimeoutConstraint(ms).
    public static PathConstraints pathConstraints = new PathConstraints(0.995, 100, 1.25, 10);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static double robotWidth(){
        return 16.4375;
    }

    public static double robotLength(){
        return 18.24;
    } //is double length from intake to center of rotation, actual length is a little shorter but this is the distance that matters for calculating
    //the distance from things because the intake is the usual reference point

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FRW")
            .rightRearMotorName("BRW")
            .leftRearMotorName("BLW")
            .leftFrontMotorName("FLW")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(76.391)
            .yVelocity(61.93);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .forwardPodY(0.79)
            .strafePodX(0.95);
}