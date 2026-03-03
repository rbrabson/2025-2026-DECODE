package org.firstinspires.ftc.teamcode.hardware;

/**
 * Holds the servo positions for the indexer in both intake and shooting modes.
 *
 * WHAT IS THE INDEXER?
 * The indexer is a rotating carousel that holds up to 3 balls. A servo rotates
 * it to position each slot either:
 *   - In front of the INTAKE (to load a ball from the field)
 *   - In front of the SHOOTER (to fire a ball at the basket)
 *
 * The intake and shoot positions are different because the intake and shooter
 * are on different sides of the indexer mechanism.
 *
 * WHY TWO CONFIGS?
 * We have two physical indexer setups on different robots. They use different
 * servo positions because the servo mounting is slightly different.
 *
 * TODO: [Student] Which robot uses CONFIG_A? Which uses CONFIG_B?
 *       How did we figure out these exact servo position values?
 *       Hint: look at ServoTester1.java -- that's how we tune them!
 *
 * TODO: [Student] What does "increment" mean? If pos1Shoot = 0.0829 and
 *       increment = 0.2084, what is pos1Shoot + increment? What slot
 *       does that correspond to?
 */
public class IndexerConfig {

    // -----------------------------------------------------------------------
    // CONFIG A: Used on the original blue-far robot (JWBF).
    // Intake positions INCREASE from slot 1 to slot 3.
    // -----------------------------------------------------------------------
    public static final IndexerConfig CONFIG_A = new IndexerConfig(
        0.3935, 0.596, 0.8045,     // intake positions for slots 1, 2, 3
        0.0829, 0.2856, 0.494,     // shoot positions for slots 1, 2, 3
        0.2084                       // distance between adjacent slots
    );

    // -----------------------------------------------------------------------
    // CONFIG B: Used on all other robots (JWRC, JWBF_Finish, JWBC, JWRF, TeleOp).
    // Intake positions DECREASE from slot 1 to slot 3.
    // -----------------------------------------------------------------------
    public static final IndexerConfig CONFIG_B = new IndexerConfig(
        0.6708, 0.4695, 0.2651,    // intake positions for slots 1, 2, 3
        0.9829, 0.7803, 0.5706,    // shoot positions for slots 1, 2, 3
        0.2085                       // distance between adjacent slots
    );

    /** Servo position to align slot 1 with the intake. */
    public final double pos1Intake;
    /** Servo position to align slot 2 with the intake. */
    public final double pos2Intake;
    /** Servo position to align slot 3 with the intake. */
    public final double pos3Intake;

    /** Servo position to align slot 1 with the shooter. */
    public final double pos1Shoot;
    /** Servo position to align slot 2 with the shooter. */
    public final double pos2Shoot;
    /** Servo position to align slot 3 with the shooter. */
    public final double pos3Shoot;

    /** Servo position distance between adjacent indexer slots. */
    public final double increment;

    /**
     * Creates an indexer configuration with the specified servo positions.
     *
     * @param p1i  intake position for slot 1
     * @param p2i  intake position for slot 2
     * @param p3i  intake position for slot 3
     * @param p1s  shoot position for slot 1
     * @param p2s  shoot position for slot 2
     * @param p3s  shoot position for slot 3
     * @param increment  distance between adjacent slots
     */
    public IndexerConfig(double p1i, double p2i, double p3i,
                         double p1s, double p2s, double p3s,
                         double increment) {
        this.pos1Intake = p1i;
        this.pos2Intake = p2i;
        this.pos3Intake = p3i;
        this.pos1Shoot = p1s;
        this.pos2Shoot = p2s;
        this.pos3Shoot = p3s;
        this.increment = increment;
    }

    /**
     * Returns the intake servo position for the given slot index.
     *
     * @param slot  the slot number (0, 1, or 2)
     * @return the servo position to align that slot with the intake
     */
    public double getIntakePosition(int slot) {
        switch (slot) {
            case 0: return pos1Intake;
            case 1: return pos2Intake;
            case 2: return pos3Intake;
            default: return pos1Intake;
        }
    }

    /**
     * Returns the shoot servo position for the given slot index.
     *
     * @param slot  the slot number (0, 1, or 2)
     * @return the servo position to align that slot with the shooter
     */
    public double getShootPosition(int slot) {
        switch (slot) {
            case 0: return pos1Shoot;
            case 1: return pos2Shoot;
            case 2: return pos3Shoot;
            default: return pos1Shoot;
        }
    }
}
