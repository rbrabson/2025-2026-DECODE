package org.firstinspires.ftc.teamcode.decode;

/**
 * Enum representing the two alliances in the FTC game: RED and BLUE.
 * Each alliance has a base position on the field defined by its X and Y coordinates.
 */
public enum Alliance {
    RED(144, 144),
    BLUE(0, 144);

    private final double baseX;
    private final double baseY;

    /**
     * Constructor for the Alliance enum.
     *
     * @param baseX The X coordinate of the alliance's base position on the field.
     * @param baseY The Y coordinate of the alliance's base position on the field.
     */
    Alliance(double baseX, double baseY) {
        this.baseX = baseX;
        this.baseY = baseY;
    }

    /**
     * Getter for the base X coordinate of the alliance's position on the field.
     *
     * @return The X coordinate of the alliance's base position.
     */
    public double getBaseX() {
        return baseX;
    }

    /**
     * Getter for the base Y coordinate of the alliance's position on the field.
     *
     * @return The Y coordinate of the alliance's base position.
     */
    public double getBaseY() {
        return baseY;
    }
}
