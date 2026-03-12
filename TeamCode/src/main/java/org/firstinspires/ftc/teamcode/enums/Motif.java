package org.firstinspires.ftc.teamcode.enums;

/**
 * Enum representing the possible motifs of the game elements.
 */
public enum Motif {
    GPP(21),
    PGP(22),
    PPG(23);

    private final int aprilTagID;
    private final Color[] motifColors;

    /**
     * Creates a Motif enum for the specified AprilTag.
     *
     * @param aprilTagID the AprilTag for the motif pattern.
     */
    Motif(int aprilTagID) {
        this.aprilTagID = aprilTagID;

        switch (aprilTagID) {
            case 21:
                motifColors = new Color[]{Color.GREEN, Color.PURPLE, Color.PURPLE};
                break;
            case 22:
                motifColors = new Color[]{Color.PURPLE, Color.GREEN, Color.PURPLE};
                break;
            case 23:
                motifColors = new Color[]{Color.PURPLE, Color.PURPLE, Color.GREEN};
                break;
            default:
                motifColors = new Color[]{Color.GREEN, Color.PURPLE, Color.PURPLE};
        }
    }

    /**
     * Gets the motif for the given AprilTag ID.
     *
     * @param id the AprilTag ID
     * @return the motif for the AprilTag ID
     */
    public static Motif fromAprilTagID(int id) {
        for (Motif motif : values()) {
            if (motif.aprilTagID == id) {
                return motif;
            }
        }
        return GPP;
    }

    /**
     * Gets the default motif (GPP).
     *
     * @return the default motif
     */
    public static Motif getDefault() {
        return Motif.GPP;
    }

    /**
     * Gets the color pattern for the motif enum.
     *
     * @return the color pattern
     */
    public Color[] colors() {
        return motifColors;
    }
}
