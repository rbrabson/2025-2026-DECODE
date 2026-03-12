package org.firstinspires.ftc.teamcode.pedro;

import androidx.annotation.NonNull;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class is responsible for creating and configuring the path follower for the autonomous period.
 * It sets up the follower constants, path constraints, localizer constants, and drivetrain constants.
 * The createFollower method initializes the follower with the specified configurations and returns
 * it for use in autonomous routines.
 */
public class PedroFollower {
    private static final FollowerConstants followerConstants = new FollowerConstants()
        .mass(10)
        .forwardZeroPowerAcceleration(-36.127)
        .lateralZeroPowerAcceleration(-72.1608)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
        .headingPIDFCoefficients(new PIDFCoefficients(0.75, 0, 0.01, 0.025))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0045, 0, 0, 0.6, 0.1))
        .centripetalScaling(0.0005);
    private static final PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(-6.5)
        .strafePodX(-6)
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("pp")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    private static final PathConstraints pathConstraints = new PathConstraints(0.99, 100, 5, 1);
    private static final MecanumConstants driveConstants = new MecanumConstants()
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

    /**
     * Creates and configures a Pinpoint localizer using the specified hardware map and localizer
     * constants.
     *
     * @param hardwareMap The hardware map used to initialize the localizer components.
     * @return A configured Localizer instance ready for use in the path follower.
     */
    @NonNull
    public static Localizer getPinpointLocalizer(@NonNull HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConstants);
    }

    /**
     * Creates and configures a fused localizer that combines the Pinpoint localizer with the
     * Limelight camera.
     *
     * @param hardwareMap The hardware map used to initialize the localizer components.
     * @param limelight The Limelight camera used for vision-based localization, which is fused with
     *                  the Pinpoint localizer
     * @return A configured Localizer instance that fuses the Pinpoint localizer with the Limelight
     *         camera for improved localization accuracy.
     */
    @NonNull
    public static FusedLocalizer getFusedLocalizer(@NonNull HardwareMap hardwareMap, @NonNull Limelight3A limelight) {
        return new FusedLocalizer(getPinpointLocalizer(hardwareMap), limelight);
    }

    /**
     * Creates and configures the path follower for autonomous routines using a fused localizer
     * that combines the Pinpoint localizer with the Limelight camera.
     *
     * @param hardwareMap The hardware map used to initialize the localizer and drivetrain
     *                    components of the follower.
     * @param limelight   The Limelight camera used for vision-based localization, which is fused
     *                    with the Pinpoint localizer for improved accuracy.
     * @param mode        The mode of the fused localizer, which determines how the data from the
     *                    Pinpoint localizer and the Limelight camera are combined for position tracking.
     * @return A configured Follower instance ready for use in autonomous routines, utilizing the
     *         fused localizer for enhanced localization accuracy and reliability.
     */
    @NonNull
    public static FusedLocalizer getFusedLocalizer(@NonNull HardwareMap hardwareMap, @NonNull Limelight3A limelight, @NonNull FusedLocalizer.Mode mode) {
        return new FusedLocalizer(getPinpointLocalizer(hardwareMap), limelight).withMode(mode);
    }

    /**
    * Creates and configures the path follower for autonomous routines using the default Pinpoint
    * localizer.
    *
    * @param hardwareMap The hardware map used to initialize the localizer and drivetrain components
    *                    of the follower.
    * @return A configured Follower instance ready for use in autonomous routines, utilizing the
    *         default Pinpoint localizer for position tracking.
    */
    @NonNull
    public static Follower create(@NonNull HardwareMap hardwareMap) {
        return create(hardwareMap, getPinpointLocalizer(hardwareMap));
    }

    /**
     * Creates and configures the path follower for autonomous routines.
     * @param hardwareMap The hardware map used to initialize the localizer and drivetrain
     *                    components of the follower.
     * @param localizer   The localizer instance used for position tracking in the follower.
     * @return A configured Follower instance ready for use in autonomous routines.
     */
    @NonNull
    public static Follower create(@NonNull HardwareMap hardwareMap, @NonNull Localizer localizer) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .setLocalizer(localizer)
            .build();
    }
}
