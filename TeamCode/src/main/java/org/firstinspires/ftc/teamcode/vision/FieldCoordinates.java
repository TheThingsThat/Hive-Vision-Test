package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Conversions between the official FTC field frame (used by the Limelight's botpose) and the Pedro
 * Pathing frame.
 * <ul>
 *   <li><b>FTC:</b> origin at the field center, meters (or inches), yaw in degrees/radians,
 *       counter-clockwise positive.</li>
 *   <li><b>Pedro:</b> origin at the bottom-left corner, inches, 0 rad = +x, counter-clockwise positive,
 *       field is 144 in square.</li>
 * </ul>
 * The formulas match the conversion published in the Pedro Pathing "Coordinates" reference page.
 */
public final class FieldCoordinates {
    private FieldCoordinates() {}

    public static final double FIELD_SIZE_IN = 144.0;
    public static final double HALF_FIELD_IN = FIELD_SIZE_IN / 2.0;

    /** FTC field pose (inches, radians) to Pedro pose. */
    public static Pose ftcToPedro(double xIn, double yIn, double headingRad) {
        return new Pose(yIn + HALF_FIELD_IN, HALF_FIELD_IN - xIn, headingRad - Math.PI / 2.0);
    }

    /** Pedro pose to FTC field pose (inches, radians). */
    public static Pose pedroToFtc(Pose pedro) {
        return new Pose(HALF_FIELD_IN - pedro.y(), pedro.x() - HALF_FIELD_IN, pedro.heading() + Math.PI / 2.0);
    }

    /** A Limelight botpose (FTC frame, meters) to a Pedro pose. Returns null if the input is null. */
    public static Pose pose3DToPedro(Pose3D botpose) {
        if (botpose == null || botpose.getPosition() == null || botpose.getOrientation() == null) return null;
        Position p = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double yaw = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        return ftcToPedro(p.x, p.y, yaw);
    }

    /** A Limelight botpose to an FTC-frame pose in inches (handy for telemetry). Returns null if the input is null. */
    public static Pose pose3DToFtcInches(Pose3D botpose) {
        if (botpose == null || botpose.getPosition() == null || botpose.getOrientation() == null) return null;
        Position p = botpose.getPosition().toUnit(DistanceUnit.INCH);
        return new Pose(p.x, p.y, botpose.getOrientation().getYaw(AngleUnit.RADIANS));
    }

    /** Pedro heading (radians) to the FTC yaw in degrees that MegaTag2 expects from {@code updateRobotOrientation}. */
    public static double pedroHeadingToFtcYawDegrees(double pedroHeadingRad) {
        return Math.toDegrees(Angle.normalizeSigned(pedroHeadingRad + Math.PI / 2.0));
    }

    /** True if the pose lies on the field, allowing {@code marginIn} inches of slack outside the walls. */
    public static boolean isOnField(Pose pedro, double marginIn) {
        return pedro.x() >= -marginIn && pedro.x() <= FIELD_SIZE_IN + marginIn
                && pedro.y() >= -marginIn && pedro.y() <= FIELD_SIZE_IN + marginIn;
    }
}
