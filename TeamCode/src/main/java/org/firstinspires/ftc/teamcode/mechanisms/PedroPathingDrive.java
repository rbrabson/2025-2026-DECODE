package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.FusedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroFollower;
import org.firstinspires.ftc.teamcode.robotcontrol.DriveController;
import org.firstinspires.ftc.teamcode.robotcontrol.VoltageCompensator;
import org.firstinspires.ftc.teamcode.utils.MathEx;

import java.util.Objects;

/**
 * This class implements a pathing drive mechanism using the Pedro Pathing library. It provides
 * methods for driving the robot, getting and setting the robot's pose, following paths, and
 * configuring the localizer mode. The class utilizes a Follower to control the robot's movement
 * based on the current pose and target paths.
 */
public class PedroPathingDrive implements Mechanism {
    public final FusedLocalizer localizer;
    private final Follower follower;
    private final Telemetry telemetry;

    private final VoltageCompensator voltageComp;
    private final DriveController driveCtrl;
    private boolean robotCentric = false;

    /**
     * Constructor for the PedroPathingDrive class. It initializes the localizer and follower using
     * the provided hardware map and limelight, and sets up telemetry for debugging and monitoring.
     *
     * @param hardwareMap The hardware map for accessing robot hardware components.
     * @param limelight   The Limelight3A instance for vision processing and localization.
     * @param telemetry   The telemetry instance for sending data to the driver station for
     *                    debugging and monitoring.
     */
    public PedroPathingDrive(HardwareMap hardwareMap, Limelight3A limelight, Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap);
        this.localizer = PedroFollower.getFusedLocalizer(map, limelight, FusedLocalizer.Mode.TELEOP);
        follower = PedroFollower.create(map, localizer);
        this.telemetry = telemetry;
        driveCtrl = new DriveController();
        voltageComp = new VoltageCompensator(Robot.getInstance(map, telemetry).voltageSensor);
    }

    /**
     * Drives the robot using the specified x (forward), y (strafe), and turn (rotation) values. This
     * method is typically used for teleop control, allowing the driver to manually control the robot
     * while still utilizing the localizer for accurate movement. The robotCentric parameter is set
     * to false, meaning the controls are field-centric by default.
     *
     * @param x    The forward/backward movement value, where positive is forward and negative is backward.
     * @param y    The strafe movement value, where positive is right and negative is left.
     * @param turn The rotation value, where positive is clockwise and negative is counterclockwise.
     */
    public void drive(double x, double y, double turn) {
        drive(x, y, turn, robotCentric);
    }

    /**
     * Sets the teleop drive values for the robot, allowing for manual control while still utilizing
     * the localizer for accurate movement.
     *
     * @param forward      The forward/backward movement value, where positive is forward and negative is
     *                     backward.
     * @param strafe       The strafe movement value, where positive is right and negative is left.
     * @param turn         The rotation value, where positive is clockwise and negative is counterclockwise.
     * @param robotCentric Whether the controls should be robot-centric (true) or field-centric (false).
     *                     In robot-centric mode, the controls are relative to the robot's orientation,
     *                     while in field-centric mode, the controls are relative to the field regardless
     *                     of the robot's orientation.
     */
    public void drive(double forward, double strafe, double turn, boolean robotCentric) {
        // Compensate the user input values for voltage to maintain consistent performance as the battery voltage changes
        double[] values = voltageComp.compensate(new double[]{forward, strafe, turn});

        // Update the drive controller with the compensated values and the current pose from the localizer to get the drive outputs
        DriveController.DriveOutput driveValues = driveCtrl.update(values[0], values[1], values[2], localizer.getPose());
        forward = driveValues.getX();
        strafe = driveValues.getY();
        turn = driveValues.getTurn();

        // Normalize the values to ensure that the maximum absolute value does not exceed 1
        double max = MathEx.maxAbs(1, forward, strafe, turn);
        if (max > 1) {
            forward /= max;
            strafe /= max;
            turn /= max;
        }
        follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
    }

    /**
     * Drives the robot to a specified target pose using a path generated by a Bezier line from the
     * current pose to the target pose. This method allows for smooth and accurate movement to a
     * specific location on the field, utilizing the localizer for precise control.
     * The path is created using a Bezier line, which provides a smooth trajectory for the robot
     * to follow.
     *
     * @param target The target pose to which the robot should drive, including its position (x, y)
     *               and orientation (heading).
     */
    public void driveToPose(Pose target) {
        Pose start = follower.getPose();
        Path path = new Path(
                new BezierLine(start, target)
        );
        follower.followPath(path);
    }

    /**
     * Returns the current pose of the robot as determined by the localizer.
     *
     * @return The current pose of the robot, including its position (x, y) and orientation (heading).
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Sets the current pose of the robot for the localizer. This can be used to manually update the
     * robot's pose if necessary, such as after a significant movement or if the localizer's estimate
     * is believed to be inaccurate.
     *
     * @param pose The current pose to set for the localizer, including the robot's
     *             position (x, y) and orientation (heading).
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /**
     * Sets the starting pose of the robot for the localizer. This is typically called during the
     * initialization phase of the robot to establish the initial position and orientation of the
     * robot on the field.
     *
     * @param pose The starting pose to set for the localizer, including the robot's initial
     *             position (x, y) and orientation (heading).
     */
    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    /**
     * Sets the mode of the localizer, which can affect how the localizer processes sensor data and
     * estimates the robot's pose. Different modes may be optimized for different phases of the match
     * (e.g., autonomous vs. teleop) or for different types of movement.
     *
     * @param mode The mode to set for the localizer, which can be one of the predefined modes in
     *             the FusedLocalizer.Mode enum (e.g., TELEOP, AUTONOMOUS).
     */
    public void setMode(FusedLocalizer.Mode mode) {
        localizer.withMode(mode);
    }

    /**
     * Starts the teleop drive mode, allowing for manual control of the robot while still utilizing
     * the localizer for accurate movement. This method should be called at the beginning of the
     * teleop period to enable teleop control.
     */
    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    /**
     * Sets whether the teleop drive controls should be robot-centric or field-centric. In
     * robot-centric mode, the controls are relative to the robot's orientation, while in
     * field-centric mode, the controls are relative to the field regardless of the robot's
     * orientation.
     *
     * @param robotCentric Whether the controls should be robot-centric (true) or field-centric (false).
     */
    public void setRobotCentric(boolean robotCentric) {
        this.robotCentric = robotCentric;
    }

    /**
     * Updates the state of the follower, which should be called in the main loop of the robot's
     * operation to ensure that the robot continues to follow its path and respond to teleop controls
     * as needed.
     */
    @Override
    public void update() {
        follower.update();
    }
}
