package org.firstinspires.ftc.teamcode.util;

/**
 * Shares robot state between autonomous and TeleOp OpModes.
 *
 * HOW AUTONOMOUS-TO-TELEOP HANDOFF WORKS:
 * When autonomous ends and TeleOp starts, the robot needs to "remember"
 * where it is on the field. Since each OpMode is a separate Java object,
 * we use static fields to persist data between them.
 *
 * Flow:
 *   1. Autonomous runs and tracks the robot's position using odometry
 *   2. Every loop iteration, autonomous writes the current pose here
 *   3. Autonomous ends, TeleOp starts
 *   4. TeleOp reads the last known pose from here during init()
 *   5. TeleOp sets PedroPathing's starting pose to these values
 *
 * TODO: [Student] Why do we need this class? Can't TeleOp just read
 *       the odometry directly? Hint: what happens to the Follower object
 *       when autonomous stops and TeleOp starts?
 *
 * TODO: [Student] What would happen if autonomous crashes before writing
 *       these values? What default values would TeleOp use?
 */
public class SharedState {

    /**
     * The detected motif from AprilTag detection.
     * Possible values: "GPP", "PGP", "PPG", or "" (not yet detected).
     *
     * TODO: [Student] What is a "motif" in this game? How does it affect scoring?
     */
    public static String motif = "PPG";

    /**
     * Last known X position in field inches.
     * Updated every loop iteration during autonomous.
     */
    public static double xPos = 7.5;

    /**
     * Last known Y position in field inches.
     * Updated every loop iteration during autonomous.
     */
    public static double yPos = 7.8;

    /**
     * Last known heading in radians.
     * Updated every loop iteration during autonomous.
     */
    public static double yaw = Math.toRadians(90);

    /**
     * Last known turret encoder position.
     * Used to adjust turret limits in TeleOp based on where auto left off.
     *
     * TODO: [Student] Why do the turret limits change between auto and teleop?
     *       Hint: the turret encoder resets at the start of each OpMode.
     */
    public static int turretPose = 0;

    /**
     * Resets all shared state to default values.
     * Call this if you want to start fresh (e.g., no auto was run).
     */
    public static void reset() {
        motif = "PPG";
        xPos = 7.5;
        yPos = 7.8;
        yaw = Math.toRadians(90);
        turretPose = 0;
    }
}
