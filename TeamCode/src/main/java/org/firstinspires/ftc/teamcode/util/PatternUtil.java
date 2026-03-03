package org.firstinspires.ftc.teamcode.util;

/**
 * Utility methods for working with ball pattern strings.
 *
 * WHAT IS A PATTERN?
 * The robot's indexer holds up to 3 balls. We represent what's in each slot
 * using a 3-character string:
 *   'G' = green ball in this slot
 *   'P' = purple ball in this slot
 *   'X' = empty slot (waiting for a ball)
 *
 * Examples:
 *   "XXX" = all slots empty (start of a new intake cycle)
 *   "GXX" = green ball in slot 1, slots 2 and 3 empty
 *   "GPP" = green in slot 1, purple in slots 2 and 3 (fully loaded)
 *
 * TODO: [Student] Why do we track the color of each ball? What happens during
 *       shooting if we don't know which slot has the green ball?
 */
public class PatternUtil {

    /**
     * Counts how many times a character appears in a string.
     *
     * For example: count("GPP", 'P') returns 2
     *              count("GPP", 'G') returns 1
     *              count("XXX", 'X') returns 3
     *
     * @param str         the string to search through
     * @param targetChar  the character to count
     * @return the number of occurrences of targetChar in str
     *
     * TODO: [Student] Can you think of a way to do this with a Java Stream or
     *       a built-in String method? Try: str.chars().filter(...).count()
     */
    public static int count(String str, Character targetChar) {
        int iter = 0;
        for (int i = 0; i < str.length(); i++) {
            if (str.charAt(i) == targetChar) {
                iter++;
            }
        }
        return iter;
    }

    /**
     * Replaces the character at a given index in the pattern string.
     *
     * For example: replaceAt("XGX", 0, 'P') returns "PGX"
     *              replaceAt("XGX", 2, 'G') returns "XGG"
     *
     * @param pattern  the current pattern (e.g., "XGP")
     * @param index    the position to replace (0, 1, or 2)
     * @param newChar  the character to insert (e.g., 'G' or 'P')
     * @return the updated pattern string
     *
     * TODO: [Student] Why can't we just use String.replace() here?
     *       Hint: what if the pattern is "PXP" and we want to replace index 0?
     *       String.replace('P', 'G') would replace BOTH P's!
     */
    public static String replaceAt(String pattern, int index, char newChar) {
        return pattern.substring(0, index) + newChar + pattern.substring(index + 1);
    }

    /**
     * Checks if the pattern has any empty slots remaining.
     *
     * @param pattern  the current pattern string
     * @return true if there is at least one 'X' (empty slot)
     */
    public static boolean hasEmptySlots(String pattern) {
        return pattern.indexOf('X') != -1;
    }

    /**
     * Returns the index of the next empty slot, or -1 if all slots are full.
     *
     * @param pattern  the current pattern string
     * @return index of first 'X', or -1 if no empty slots
     */
    public static int getNextEmptySlot(String pattern) {
        return pattern.indexOf('X');
    }
}
