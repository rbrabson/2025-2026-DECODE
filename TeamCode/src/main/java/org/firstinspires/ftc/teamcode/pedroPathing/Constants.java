package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(10).forwardZeroPowerAcceleration(-36.127).lateralZeroPowerAcceleration(-72.1608).translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02)).headingPIDFCoefficients(new PIDFCoefficients(0.75, 0, 0.01, 0.025)).drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0045, 0, 0, 0.6, 0.1)).centripetalScaling(0.0005);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 5, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.5)
            .strafePodX(-6)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pp")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(73.30485)
            .yVelocity(54.3355)
            .rightFrontMotorName("frm")
            .rightRearMotorName("brm")
            .leftRearMotorName("blm")
            .leftFrontMotorName("flm")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
