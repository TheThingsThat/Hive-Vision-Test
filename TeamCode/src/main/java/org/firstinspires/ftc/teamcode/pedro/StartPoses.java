package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.planning.PlannerConstants;

/**
 * Exact starting poses for a robot whose back is flush against a field wall, facing straight out. With a
 * known start the odometry needs no external correction for a short run, so AprilTags are not required.
 * <p>
 * Pedro coordinates: origin is the corner on the red alliance side that is on your RIGHT when you stand at
 * the red wall looking across the field. +x runs away from the red wall, +y runs to your left, heading 0
 * faces the blue wall, counter-clockwise is positive. The field is 144 in square.
 */
public final class StartPoses {
    private StartPoses() {}

    /** The four walls, named as seen from the red alliance wall. */
    public enum Wall {
        /** The red alliance wall (x = 0). Facing away from it is heading 0. */
        RED,
        /** The blue alliance wall (x = 144). Facing away from it is heading 180 deg. */
        BLUE,
        /** The side wall on your right when standing at the red wall (y = 0). Facing away is 90 deg. */
        RIGHT_SIDE,
        /** The side wall on your left when standing at the red wall (y = 144). Facing away is 270 deg. */
        LEFT_SIDE
    }

    /**
     * Robot center with its back flush against {@code wall}, facing straight away from it, positioned
     * {@code alongIn} inches along the wall measured in the +x or +y direction.
     */
    public static Pose againstWall(Wall wall, double alongIn) {
        double back = PlannerConstants.ROBOT_LENGTH_IN / 2.0; // wall to robot center
        double field = PlannerConstants.FIELD_SIZE_IN;
        switch (wall) {
            case RED:
                return new Pose(back, alongIn, 0);
            case BLUE:
                return new Pose(field - back, alongIn, Math.PI);
            case RIGHT_SIDE:
                return new Pose(alongIn, back, Math.PI / 2);
            case LEFT_SIDE:
            default:
                return new Pose(alongIn, field - back, -Math.PI / 2);
        }
    }

    /** Robot centered on {@code wall}, back flush against it, facing straight out. */
    public static Pose wallCenter(Wall wall) {
        return againstWall(wall, PlannerConstants.FIELD_SIZE_IN / 2.0);
    }
}
