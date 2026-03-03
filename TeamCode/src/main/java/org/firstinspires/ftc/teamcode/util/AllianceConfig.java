package org.firstinspires.ftc.teamcode.util;

/**
 * Stores alliance-specific constants for turret targeting.
 *
 * The FTC field is 144 x 144 inches. Each alliance has a basket (target)
 * at a different corner of the field. The turret must aim at the correct
 * basket depending on which alliance we are playing for.
 *
 * Field layout (top-down view):
 *
 *   (0,144) BLUE BASKET -------- RED BASKET (144,144)
 *         |                                  |
 *         |            FIELD                 |
 *         |          (72, 72)                |
 *         |                                  |
 *   (0,0) ---------------------------(144,0)
 *
 * TODO: [Student] Look at the FTC field diagram for this season.
 *       Which corner is the blue basket in? Which is the red basket?
 *       Verify that these coordinates match the actual field layout.
 */
public enum AllianceConfig {

    /**
     * Blue alliance: turret aims at the blue basket at field coordinate (0, 144).
     *
     * TODO: [Student] What is at position (0, 144) on the field?
     */
    BLUE(0, 144),

    /**
     * Red alliance: turret aims at the red basket at field coordinate (144, 144).
     *
     * TODO: [Student] What is at position (144, 144) on the field?
     */
    RED(144, 144);

    /** The X coordinate of the target basket on the field (in inches). */
    public final double targetX;

    /** The Y coordinate of the target basket on the field (in inches). */
    public final double targetY;

    AllianceConfig(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
    }
}
