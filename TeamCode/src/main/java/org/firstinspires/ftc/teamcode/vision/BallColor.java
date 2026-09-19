package org.firstinspires.ftc.teamcode.vision;

/**
 * Ball classes exactly as Hive-Vision's {@code labels.txt} lists them.
 * The declaration order matches the model's class ids (0 = yellow, 1 = red, 2 = blue).
 */
public enum BallColor {
    YELLOW("yellow_pollen", 0),
    RED("red_nectar", 1),
    BLUE("blue_nectar", 2);

    /** Label string reported by the Limelight neural detector. */
    public final String label;

    /** Class id reported by the Limelight neural detector. */
    public final int classId;

    BallColor(String label, int classId) {
        this.label = label;
        this.classId = classId;
    }

    /** Matches the exact Hive-Vision label, or a label that merely starts with the color name (e.g. "yellow"). */
    public static BallColor fromLabel(String label) {
        if (label == null) return null;
        String l = label.trim().toLowerCase();
        for (BallColor c : values()) {
            if (c.label.equals(l) || l.startsWith(c.name().toLowerCase())) return c;
        }
        return null;
    }

    public static BallColor fromClassId(int classId) {
        for (BallColor c : values()) {
            if (c.classId == classId) return c;
        }
        return null;
    }
}
