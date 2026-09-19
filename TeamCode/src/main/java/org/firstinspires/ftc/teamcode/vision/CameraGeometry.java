package org.firstinspires.ftc.teamcode.vision;

/**
 * Converts Limelight target angles into a floor position in the robot frame using the camera mount
 * geometry in {@link LimelightConstants}.
 */
public final class CameraGeometry {
    private CameraGeometry() {}

    /**
     * Intersects the camera ray through the target with the horizontal plane at the ball's center height.
     *
     * @param txDeg horizontal angle from the crosshair, degrees, positive = right
     * @param tyDeg vertical angle from the crosshair, degrees, positive = up
     * @return {@code {forwardIn, lateralIn}} in the robot frame (+forward, +left), or {@code {NaN, NaN}}
     *         when the ray never reaches that plane (target above the horizon).
     */
    public static double[] floorPoint(double txDeg, double tyDeg) {
        double pitch = Math.toRadians(LimelightConstants.CAMERA_PITCH_DOWN_DEG);
        double tanTx = Math.tan(Math.toRadians(txDeg));
        double tanTy = Math.tan(Math.toRadians(tyDeg));

        // Ray direction in the robot frame. The camera's optical axis is pitched down by `pitch`;
        // the camera's "right" axis stays horizontal.
        double dirForward = Math.cos(pitch) + Math.sin(pitch) * tanTy;
        double dirUp = -Math.sin(pitch) + Math.cos(pitch) * tanTy;
        double dirRight = tanTx;

        double drop = LimelightConstants.CAMERA_HEIGHT_IN - LimelightConstants.BALL_DIAMETER_IN / 2.0;
        if (drop <= 0 || dirUp >= -1e-6) {
            return new double[] {Double.NaN, Double.NaN};
        }

        double t = drop / -dirUp;
        double forward = LimelightConstants.CAMERA_FORWARD_OFFSET_IN + t * dirForward;
        double lateral = LimelightConstants.CAMERA_LATERAL_OFFSET_IN - t * dirRight;
        return new double[] {forward, lateral};
    }
}
