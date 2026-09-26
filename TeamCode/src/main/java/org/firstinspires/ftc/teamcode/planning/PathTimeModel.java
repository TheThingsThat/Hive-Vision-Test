package org.firstinspires.ftc.teamcode.planning;

import com.pedropathing.math.Vector2D;

import java.util.List;

/**
 * Samples straight and cubic-Bezier pieces and estimates how long the robot takes to drive them.
 * <p>
 * Speed along a piece is limited by top speed, a centripetal-acceleration limit (so tight curves are slow)
 * and, when the heading follows the tangent, a turn-rate limit. {@link #uncoupledTime} ignores
 * acceleration between pieces so costs are additive for the dynamic program; {@link #coupledTime}
 * applies acceleration and braking limits across a whole path for the final ranking.
 */
final class PathTimeModel {
    private PathTimeModel() {}

    /** One sampled piece of a path. */
    static final class Piece {
        final boolean isLine;
        final Vector2D p0, p1, p2, p3;
        final int n;              // number of intervals; there are n + 1 sample points
        final double[] x, y;      // sample positions
        final double[] theta;     // tangent direction at each sample
        final double[] kappa;     // signed curvature at each sample
        final double[] ds;        // arc length of interval i (i < n)
        final double length;
        /** Number of direction reversals (cusps) along the piece; each one forces a stop and restart. */
        int cusps = 0;
        /** False when the heading is interpolated linearly from {@code headingStart} to {@code headingEnd} instead of following the tangent. */
        boolean tangentHeading = true;
        /** Robot heading at the start / end of a non-tangent piece. */
        double headingStart = 0;
        double headingEnd = 0;
        /**
         * For a non-tangent piece: fraction of the arc length over which the heading is held at
         * {@code headingStart} before rotating, so the robot clears nearby walls before it turns.
         */
        double holdFraction = 0;
        /** Heading change that must be completed during this piece when {@code !tangentHeading}. */
        double turnRad = 0;

        private Piece(boolean isLine, Vector2D p0, Vector2D p1, Vector2D p2, Vector2D p3, int n) {
            this.isLine = isLine;
            this.p0 = p0;
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
            this.n = n;
            x = new double[n + 1];
            y = new double[n + 1];
            theta = new double[n + 1];
            kappa = new double[n + 1];
            ds = new double[n];
            double len = 0;
            for (int i = 0; i <= n; i++) {
                double t = (double) i / n;
                sample(t, i);
                if (i > 0) {
                    ds[i - 1] = Math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
                    len += ds[i - 1];
                }
            }
            length = len;
            // A cubic Bezier reverses direction (cusp) when consecutive tangents point opposite ways. The
            // curvature formula reports zero for collinear control points, so detect it explicitly.
            for (int i = 1; i <= n; i++) {
                double d = Math.cos(theta[i] - theta[i - 1]);
                if (d < 0) {
                    cusps++;
                    kappa[i] = Double.POSITIVE_INFINITY;
                }
            }
        }

        private void sample(double t, int i) {
            if (isLine) {
                x[i] = p0.x() + (p3.x() - p0.x()) * t;
                y[i] = p0.y() + (p3.y() - p0.y()) * t;
                theta[i] = Math.atan2(p3.y() - p0.y(), p3.x() - p0.x());
                kappa[i] = 0;
                return;
            }
            double u = 1 - t;
            double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
            x[i] = b0 * p0.x() + b1 * p1.x() + b2 * p2.x() + b3 * p3.x();
            y[i] = b0 * p0.y() + b1 * p1.y() + b2 * p2.y() + b3 * p3.y();
            double dx = 3 * u * u * (p1.x() - p0.x()) + 6 * u * t * (p2.x() - p1.x()) + 3 * t * t * (p3.x() - p2.x());
            double dy = 3 * u * u * (p1.y() - p0.y()) + 6 * u * t * (p2.y() - p1.y()) + 3 * t * t * (p3.y() - p2.y());
            double ddx = 6 * u * (p2.x() - 2 * p1.x() + p0.x()) + 6 * t * (p3.x() - 2 * p2.x() + p1.x());
            double ddy = 6 * u * (p2.y() - 2 * p1.y() + p0.y()) + 6 * t * (p3.y() - 2 * p2.y() + p1.y());
            double mag = Math.hypot(dx, dy);
            if (mag < 1e-9) {
                // Degenerate tangent (only at the ends when a handle collapses); fall back to the chord.
                theta[i] = Math.atan2(p3.y() - p0.y(), p3.x() - p0.x());
                kappa[i] = 0;
            } else {
                theta[i] = Math.atan2(dy, dx);
                kappa[i] = (dx * ddy - dy * ddx) / (mag * mag * mag);
            }
        }
    }

    static Vector2D unit(double angle) {
        return Vector2D.cartesian(Math.cos(angle), Math.sin(angle));
    }

    /** Cubic Bezier from {@code a} leaving along {@code tanA} to {@code b} arriving along {@code tanB}. */
    static Piece bezier(Vector2D a, double tanA, Vector2D b, double tanB) {
        double h = Math.max(PlannerConstants.MIN_HANDLE_IN, PlannerConstants.HANDLE_FRACTION * a.distance(b));
        Vector2D p1 = a.plus(unit(tanA).times(h));
        Vector2D p2 = b.minus(unit(tanB).times(h));
        return new Piece(false, a, p1, p2, b, Math.max(4, PlannerConstants.SAMPLES_PER_SEGMENT));
    }

    static Piece line(Vector2D a, Vector2D b) {
        return new Piece(true, a, a, b, b, 4);
    }

    /** Curvature-limited speed at a sample. */
    static double speedLimit(double kappa, boolean tangentHeading) {
        double v = PlannerConstants.MAX_VELOCITY_IN_S;
        double k = Math.abs(kappa);
        if (Double.isInfinite(k)) return 1.0; // cusp: effectively stopped
        if (k > 1e-6) {
            v = Math.min(v, Math.sqrt(PlannerConstants.MAX_LATERAL_ACCEL_IN_S2 / k));
            if (tangentHeading) v = Math.min(v, PlannerConstants.MAX_TURN_RATE_RAD_S / k);
        }
        return Math.max(v, 1.0);
    }

    /** Drive time of one piece ignoring acceleration limits, plus any turn-in-place time it must absorb. */
    static double uncoupledTime(Piece p) {
        double t = 0;
        for (int i = 0; i < p.n; i++) {
            double v = 0.5 * (speedLimit(p.kappa[i], p.tangentHeading) + speedLimit(p.kappa[i + 1], p.tangentHeading));
            t += p.ds[i] / v;
        }
        if (!p.tangentHeading) {
            t = Math.max(t, turnTime(p));
        }
        return t + p.cusps * cuspPenalty();
    }

    /** Time the heading rotation of a non-tangent piece needs, given that it only happens after the hold. */
    static double turnTime(Piece p) {
        double share = Math.max(0.05, 1.0 - p.holdFraction);
        return p.turnRad / PlannerConstants.TURN_IN_PLACE_RATE_RAD_S / share;
    }

    /**
     * Arc-length fraction at which the piece is far enough from every wall that the footprint can rotate
     * freely (half the robot diagonal). 0 if clear from the start, 1 if never clear.
     */
    static double clearanceFraction(Piece p) {
        double halfDiag = Math.hypot(PlannerConstants.ROBOT_LENGTH_IN, PlannerConstants.ROBOT_WIDTH_IN) / 2.0;
        double field = PlannerConstants.FIELD_SIZE_IN;
        if (p.length < 1e-9) return 0;
        double cum = 0;
        for (int i = 0; i <= p.n; i++) {
            if (i > 0) cum += p.ds[i - 1];
            double d = Math.min(Math.min(p.x[i], field - p.x[i]), Math.min(p.y[i], field - p.y[i]));
            if (d >= halfDiag) return Math.min(1.0, cum / p.length);
        }
        return 1.0;
    }

    /** Time lost by braking to a stop and accelerating again, seconds. */
    static double cuspPenalty() {
        double v = PlannerConstants.MAX_VELOCITY_IN_S;
        return 0.5 * (v / PlannerConstants.MAX_ACCEL_IN_S2 + v / PlannerConstants.MAX_DECEL_IN_S2);
    }

    /** Heading Pedro will command at sample {@code i}: the tangent, or the linear interpolation for free pieces. */
    static double headingAt(Piece p, int i, double cumLength) {
        if (p.tangentHeading) return p.theta[i];
        double frac = p.length > 1e-9 ? cumLength / p.length : 1.0;
        if (p.holdFraction < 1.0) {
            frac = frac <= p.holdFraction ? 0 : (frac - p.holdFraction) / (1.0 - p.holdFraction);
        }
        double delta = Math.atan2(Math.sin(p.headingEnd - p.headingStart), Math.cos(p.headingEnd - p.headingStart));
        return p.headingStart + delta * frac;
    }

    /** Largest distance the heading-rotated footprint pokes past a wall along this piece, inches (0 if none). */
    static double wallIntrusion(Piece p) {
        double halfL = PlannerConstants.ROBOT_LENGTH_IN / 2.0;
        double halfW = PlannerConstants.ROBOT_WIDTH_IN / 2.0;
        double field = PlannerConstants.FIELD_SIZE_IN;
        double worst = 0;
        double cum = 0;
        for (int i = 0; i <= p.n; i++) {
            if (i > 0) cum += p.ds[i - 1];
            double h = headingAt(p, i, cum);
            double c = Math.abs(Math.cos(h));
            double sn = Math.abs(Math.sin(h));
            double ex = halfL * c + halfW * sn; // footprint half-extent along x for this heading
            double ey = halfL * sn + halfW * c;
            double out = 0;
            out = Math.max(out, ex - p.x[i]);
            out = Math.max(out, p.x[i] + ex - field);
            out = Math.max(out, ey - p.y[i]);
            out = Math.max(out, p.y[i] + ey - field);
            worst = Math.max(worst, out);
        }
        return worst;
    }

    /**
     * Seconds of penalty for the footprint leaving the field along this piece: linear below
     * {@code MAX_WALL_INTRUSION_IN}, infinite (infeasible) beyond it.
     */
    static double outOfFieldPenalty(Piece p) {
        double worst = wallIntrusion(p);
        if (worst > PlannerConstants.MAX_WALL_INTRUSION_IN) return Double.POSITIVE_INFINITY;
        return worst * PlannerConstants.OUT_OF_FIELD_PENALTY_S_PER_IN;
    }

    /**
     * Drive time for a whole path with acceleration and braking limits applied across piece boundaries.
     *
     * @param v0        current speed at the start, in/s
     * @param stopAtEnd whether the robot must come to rest at the end
     */
    static double coupledTime(List<Piece> pieces, double v0, boolean stopAtEnd) {
        int points = 0;
        for (Piece p : pieces) points += p.n + 1;
        if (points < 2) return 0;

        double[] vlim = new double[points];
        double[] ds = new double[points - 1];
        int idx = 0;
        for (Piece p : pieces) {
            for (int i = 0; i <= p.n; i++) {
                vlim[idx] = speedLimit(p.kappa[i], p.tangentHeading);
                if (idx < points - 1) ds[idx] = (i < p.n) ? p.ds[i] : 0; // zero-length join between pieces
                idx++;
            }
        }

        double[] v = new double[points];
        v[0] = Math.min(Math.max(v0, 0), vlim[0]);
        for (int i = 0; i < points - 1; i++) {
            v[i + 1] = Math.min(vlim[i + 1], Math.sqrt(v[i] * v[i] + 2 * PlannerConstants.MAX_ACCEL_IN_S2 * ds[i]));
        }
        if (stopAtEnd) v[points - 1] = 0;
        for (int i = points - 2; i >= 0; i--) {
            v[i] = Math.min(v[i], Math.sqrt(v[i + 1] * v[i + 1] + 2 * PlannerConstants.MAX_DECEL_IN_S2 * ds[i]));
        }

        double total = 0;
        idx = 0;
        for (Piece p : pieces) {
            double pieceTime = 0;
            for (int i = 0; i <= p.n; i++) {
                if (idx < points - 1) {
                    double vm = Math.max(0.5 * (v[idx] + v[idx + 1]), 1e-3);
                    pieceTime += ds[idx] / vm;
                }
                idx++;
            }
            if (!p.tangentHeading) {
                pieceTime = Math.max(pieceTime, turnTime(p));
            }
            total += pieceTime;
        }
        return total;
    }
}
