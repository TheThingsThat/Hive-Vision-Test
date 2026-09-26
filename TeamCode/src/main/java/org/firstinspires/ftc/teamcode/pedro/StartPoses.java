package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.planning.PlannerConstants;

/**
 * Exact starting poses for a robot pressed into a field corner. With a known start the odometry needs no
 * external correction for a short run, so AprilTags are not required.
 * <p>
 * Pedro coordinates: origin is the corner on the red alliance side that is on your RIGHT when you stand at
 * the red wall looking across the field. +x runs away from the red wall, +y runs to your left, heading 0
 * faces the blue wall, counter-clockwise is positive. The field is 144 in square.
 */
public final class StartPoses {
    private StartPoses() {}

    /** The four corners, named as seen from the red alliance wall. */
    public enum Corner {
        /** Red side, right: the Pedro origin (0, 0). */
        NEAR_RIGHT(0, 0),
        /** Red side, left: (0, 144). */
        NEAR_LEFT(0, PlannerConstants.FIELD_SIZE_IN),
        /** Blue side, right: (144, 0). */
        FAR_RIGHT(PlannerConstants.FIELD_SIZE_IN, 0),
        /** Blue side, left: (144, 144). */
        FAR_LEFT(PlannerConstants.FIELD_SIZE_IN, PlannerConstants.FIELD_SIZE_IN);

        public final double x;
        public final double y;

        Corner(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * Robot center when its footprint is pushed fully into {@code corner} with the given heading.
     * Works for any heading; for the four cardinal headings the robot's back and one side touch the walls.
     */
    public static Pose corner(Corner corner, double headingRad) {
        double halfL = PlannerConstants.ROBOT_LENGTH_IN / 2.0;
        double halfW = PlannerConstants.ROBOT_WIDTH_IN / 2.0;
        double c = Math.abs(Math.cos(headingRad));
        double s = Math.abs(Math.sin(headingRad));
        double ex = halfL * c + halfW * s; // footprint half-extent along x for this heading
        double ey = halfL * s + halfW * c;
        double field = PlannerConstants.FIELD_SIZE_IN;
        double x = corner.x == 0 ? ex : field - ex;
        double y = corner.y == 0 ? ey : field - ey;
        return new Pose(x, y, headingRad);
    }

    /** {@link #corner(Corner, double)} with heading in degrees. */
    public static Pose cornerDegrees(Corner corner, double headingDeg) {
        return corner(corner, Math.toRadians(headingDeg));
    }

    /** A pose moved {@code distance} inches diagonally toward the field center, same heading. Used to clear the walls before rotating. */
    public static Pose towardCenter(Pose from, double distance) {
        double half = PlannerConstants.FIELD_SIZE_IN / 2.0;
        double dx = from.x() < half ? 1 : -1;
        double dy = from.y() < half ? 1 : -1;
        double step = distance / Math.sqrt(2);
        return new Pose(from.x() + dx * step, from.y() + dy * step, from.heading());
    }
}
