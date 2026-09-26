package org.firstinspires.ftc.teamcode.planning;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.planning.PathTimeModel.Piece;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Chooses which balls to collect, in what order, and from which direction, so the whole pickup run takes
 * the least time, then turns the result into a Pedro path.
 * <p>
 * The selection and the path shape are optimized together in three stages:
 * <ol>
 *   <li><b>Discrete search.</b> Every ordered sequence of {@code BALLS_TO_COLLECT} balls is enumerated.
 *       For each one a dynamic program picks the approach heading at every ball from
 *       {@code HEADING_BINS} options, using additive segment times from a curvature-limited speed model.
 *       Prefix costs are shared across sequences and branches worse than the current top set are pruned.</li>
 *   <li><b>Coupled re-scoring.</b> The best {@code TOP_CANDIDATES} solutions are re-timed with acceleration
 *       and braking limits applied across the whole path.</li>
 *   <li><b>Continuous refinement.</b> Each candidate's headings and departure directions are polished by
 *       coordinate descent on the coupled time, and the overall best becomes the plan.</li>
 * </ol>
 * A pickup is modelled as the robot's capture point (a fixed distance ahead of its center along the
 * direction of travel) passing over the ball, with the ball on the robot's centerline. That is enforced
 * by construction: each ball's waypoint is the ball position pulled back along the approach heading, the
 * path arrives there tangent to that heading, and the last few inches are straight.
 * <p>
 * Leaving a pickup, the robot either keeps following the path tangent (fast, smooth) or, when that would
 * run it into a wall or a sharp reversal, departs in a free direction while its heading rotates linearly
 * to the next approach heading (the drivetrain is holonomic, so motion and heading can differ). Such a
 * segment, and the very first one, holds its heading until the footprint is clear of the walls before it
 * starts rotating, so a robot starting flush in a corner does not scrape its way out.
 */
public class PickupPlanner {

    private static final double FREE_DEPARTURE_TRIGGER_RAD = Math.toRadians(60);
    private static final double[] FREE_DEPARTURE_OFFSETS = {0, Math.toRadians(45), Math.toRadians(-45)};

    /** One discrete solution from stage 1, later refined in place. */
    private static final class Candidate {
        final int[] order;
        final int[] headingBin;
        final int startTanBin;
        final double dpCost;
        double[] headings;      // approach direction at each ball
        boolean[] free;         // free[s]: segment s (s >= 1) departs non-tangentially
        double[] depart;        // departure direction for free segments
        double startTangent;
        double cost = Double.POSITIVE_INFINITY;
        double timeOnly;
        List<Piece> pieces;

        Candidate(int[] order, int[] headingBin, int startTanBin, double dpCost) {
            this.order = order;
            this.headingBin = headingBin;
            this.startTanBin = startTanBin;
            this.dpCost = dpCost;
        }
    }

    private long sequencesEvaluated;

    // Search state (valid during plan())
    private int n, m, k;
    private double[] bins;
    private Vector2D[] pos;
    private double[][] startCost;
    private int[][] startTan;
    private double[][][][] pair;
    private boolean[][][][] pairFree;
    private double[][][][] pairDepart;
    private double[][] best;
    private int[][] back;
    private int[] order;
    private boolean[] used;
    private List<Candidate> top;
    private double worst;

    /**
     * @param robotPose  current robot pose (Pedro coordinates)
     * @param robotSpeed current robot speed, in/s (0 if stationary)
     * @param known      candidate balls in field coordinates
     * @return the plan, or null if there are no reachable balls
     */
    public PickupPlan plan(Pose robotPose, double robotSpeed, List<FieldBall> known) {
        long t0 = System.nanoTime();
        sequencesEvaluated = 0;

        List<FieldBall> balls = nearest(known, robotPose.toVector2D(), PlannerConstants.MAX_CANDIDATE_BALLS);
        n = balls.size();
        if (n == 0) return null;
        m = Math.min(PlannerConstants.BALLS_TO_COLLECT, n);
        k = Math.max(4, PlannerConstants.HEADING_BINS);

        bins = new double[k];
        for (int i = 0; i < k; i++) bins[i] = 2 * Math.PI * i / k;
        pos = new Vector2D[n];
        for (int i = 0; i < n; i++) pos[i] = balls.get(i).position();

        buildTables(robotPose);

        // ---- Stage 1b: enumerate sequences with an incremental Viterbi over headings.
        // Segments that would put the robot through a wall cost +infinity, so a ball that cannot be reached
        // legally never appears in a finite-cost sequence. If no sequence of m balls is feasible, plan for
        // fewer rather than fail.
        top = new ArrayList<>();
        while (m >= 1) {
            best = new double[m][k];
            back = new int[m][k];
            order = new int[m];
            used = new boolean[n];
            top.clear();
            worst = Double.POSITIVE_INFINITY;
            dfs(0, -1);
            if (!top.isEmpty()) break;
            m--;
        }
        if (top.isEmpty()) return null;

        // ---- Stage 2 + 3: coupled re-scoring and continuous refinement
        double binWidth = 2 * Math.PI / k;
        Candidate winner = null;
        for (Candidate c : top) {
            refine(c, robotPose, robotSpeed, binWidth);
            if (winner == null || c.cost < winner.cost) winner = c;
        }

        // ---- Build the plan
        List<FieldBall> chosen = new ArrayList<>();
        List<Pose> pickupPoses = new ArrayList<>();
        for (int i = 0; i < m; i++) {
            chosen.add(balls.get(winner.order[i]));
            Vector2D wp = pickupPoint(pos[winner.order[i]], winner.headings[i]);
            pickupPoses.add(new Pose(wp.x(), wp.y(), pickupHeading(winner.headings[i])));
        }
        Path path = toPedroPath(winner.pieces);
        long ms = (System.nanoTime() - t0) / 1_000_000L;
        return new PickupPlan(chosen, pickupPoses, winner.headings.clone(), winner.startTangent, path,
                winner.timeOnly, ms, sequencesEvaluated);
    }

    public long sequencesEvaluated() {
        return sequencesEvaluated;
    }

    // ------------------------------------------------------------------ geometry

    static double pickupHeading(double approach) {
        return Angle.normalize(approach + (PlannerConstants.INTAKE_AT_REAR ? Math.PI : 0));
    }

    /** Robot center position when a ball at {@code ball} reaches the capture point, approaching along {@code approach}. */
    static Vector2D pickupPoint(Vector2D ball, double approach) {
        return ball.minus(PathTimeModel.unit(approach).times(PlannerConstants.PICKUP_POINT_OFFSET_IN));
    }

    /** Where the straight run-in to a pickup begins. */
    static Vector2D runInPoint(Vector2D ball, double approach) {
        return pickupPoint(ball, approach).minus(PathTimeModel.unit(approach).times(Math.max(0, PlannerConstants.STRAIGHT_APPROACH_IN)));
    }

    /**
     * Appends the pieces that take the robot from {@code from}, initially moving along {@code departDir}, to
     * the pickup of {@code ball} along {@code approach}: a Bezier into a straight run-in.
     *
     * @param freeHeading  false: heading follows the tangent (departDir must equal the previous approach);
     *                     true: heading rotates linearly from {@code headingStart} to the pickup heading.
     */
    private static void appendApproach(List<Piece> out, Vector2D from, double departDir, Vector2D ball,
                                       double approach, boolean freeHeading, double headingStart) {
        Vector2D wp = pickupPoint(ball, approach);
        Vector2D bezEnd = runInPoint(ball, approach);
        double headingEnd = pickupHeading(approach);
        double turn = freeHeading ? Math.abs(Angle.normalizeSigned(headingEnd - headingStart)) : 0;
        boolean turnAssigned = false;

        if (from.distance(bezEnd) > 1e-3) {
            Piece b = PathTimeModel.bezier(from, departDir, bezEnd, approach);
            if (freeHeading) {
                b.tangentHeading = false;
                b.headingStart = headingStart;
                b.headingEnd = headingEnd;
                b.turnRad = turn;
                // Hold the heading until the footprint is clear of the walls, then rotate. If the piece never
                // gets clear, rotate over the whole piece and let the wall check judge it.
                double hold = PathTimeModel.clearanceFraction(b);
                b.holdFraction = hold < 1.0 ? hold : 0;
                turnAssigned = true;
            }
            out.add(b);
        }
        if (PlannerConstants.STRAIGHT_APPROACH_IN > 0) {
            Piece l = PathTimeModel.line(bezEnd, wp);
            if (freeHeading && !turnAssigned) {
                l.tangentHeading = false;
                l.headingStart = headingStart;
                l.headingEnd = headingEnd;
                l.turnRad = turn;
            }
            out.add(l);
        }
    }

    private static double segmentCost(Vector2D from, double departDir, Vector2D ball, double approach,
                                      boolean freeHeading, double headingStart) {
        List<Piece> pieces = new ArrayList<>(2);
        appendApproach(pieces, from, departDir, ball, approach, freeHeading, headingStart);
        double cost = 0;
        for (Piece p : pieces) cost += PathTimeModel.uncoupledTime(p) + PathTimeModel.outOfFieldPenalty(p);
        return cost;
    }

    private List<Piece> buildPieces(Pose robotPose, Candidate c) {
        List<Piece> pieces = new ArrayList<>(2 * m + 1);
        Vector2D from = robotPose.toVector2D();
        double prevHeading = robotPose.heading();
        for (int s = 0; s < m; s++) {
            Vector2D ball = pos[c.order[s]];
            if (s == 0) {
                appendApproach(pieces, from, c.startTangent, ball, c.headings[0], true, prevHeading);
            } else if (c.free[s]) {
                appendApproach(pieces, from, c.depart[s], ball, c.headings[s], true, prevHeading);
            } else {
                appendApproach(pieces, from, c.headings[s - 1], ball, c.headings[s], false, prevHeading);
            }
            from = pickupPoint(ball, c.headings[s]);
            prevHeading = pickupHeading(c.headings[s]);
        }
        if (PlannerConstants.FINAL_OVERRUN_IN > 0) {
            double lastApproach = c.headings[m - 1];
            pieces.add(PathTimeModel.line(from, from.plus(PathTimeModel.unit(lastApproach).times(PlannerConstants.FINAL_OVERRUN_IN))));
        }
        return pieces;
    }

    // ------------------------------------------------------------------ stage 1a: cost tables

    private void buildTables(Pose robotPose) {
        Vector2D start = robotPose.toVector2D();
        double robotHeading = robotPose.heading();

        // From the robot: initial direction of travel is free, heading rotates to the first approach heading.
        startCost = new double[n][k];
        startTan = new int[n][k];
        for (int j = 0; j < n; j++) {
            for (int kj = 0; kj < k; kj++) {
                double bestC = Double.POSITIVE_INFINITY;
                int bestS = 0;
                for (int ks = 0; ks < k; ks++) {
                    double c = segmentCost(start, bins[ks], pos[j], bins[kj], true, robotHeading);
                    if (c < bestC) {
                        bestC = c;
                        bestS = ks;
                    }
                }
                startCost[j][kj] = bestC;
                startTan[j][kj] = bestS;
            }
        }

        // Ball to ball: tangent departure first; free departures when that is blocked or a sharp reversal.
        pair = new double[n][k][n][k];
        pairFree = new boolean[n][k][n][k];
        pairDepart = new double[n][k][n][k];
        for (int i = 0; i < n; i++) {
            for (int ki = 0; ki < k; ki++) {
                Vector2D from = pickupPoint(pos[i], bins[ki]);
                double headingHere = pickupHeading(bins[ki]);
                for (int j = 0; j < n; j++) {
                    if (i == j) continue;
                    for (int kj = 0; kj < k; kj++) {
                        double bestC = segmentCost(from, bins[ki], pos[j], bins[kj], false, headingHere);
                        boolean bestFree = false;
                        double bestDep = bins[ki];

                        double chord = runInPoint(pos[j], bins[kj]).minus(from).theta();
                        boolean sharp = Math.abs(Angle.normalizeSigned(chord - bins[ki])) > FREE_DEPARTURE_TRIGGER_RAD;
                        if (Double.isInfinite(bestC) || sharp) {
                            for (double off : FREE_DEPARTURE_OFFSETS) {
                                double dep = Angle.normalize(chord + off);
                                double c = segmentCost(from, dep, pos[j], bins[kj], true, headingHere);
                                if (c < bestC) {
                                    bestC = c;
                                    bestFree = true;
                                    bestDep = dep;
                                }
                            }
                            double reverse = Angle.normalize(bins[ki] + Math.PI);
                            double c = segmentCost(from, reverse, pos[j], bins[kj], true, headingHere);
                            if (c < bestC) {
                                bestC = c;
                                bestFree = true;
                                bestDep = reverse;
                            }
                        }
                        pair[i][ki][j][kj] = bestC;
                        pairFree[i][ki][j][kj] = bestFree;
                        pairDepart[i][ki][j][kj] = bestDep;
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------ stage 1b: sequence search

    private void dfs(int depth, int last) {
        if (depth == m) {
            sequencesEvaluated++;
            double[] fin = best[m - 1];
            double c = Double.POSITIVE_INFINITY;
            int arg = 0;
            for (int kk = 0; kk < k; kk++) {
                if (fin[kk] < c) {
                    c = fin[kk];
                    arg = kk;
                }
            }
            if (c < worst) addCandidate(c, arg);
            return;
        }
        for (int j = 0; j < n; j++) {
            if (used[j]) continue;
            double[] nb = best[depth];
            int[] bk = back[depth];
            double minc = Double.POSITIVE_INFINITY;
            if (depth == 0) {
                for (int kj = 0; kj < k; kj++) {
                    nb[kj] = startCost[j][kj];
                    bk[kj] = startTan[j][kj];
                    if (nb[kj] < minc) minc = nb[kj];
                }
            } else {
                double[] prev = best[depth - 1];
                double[][][] row = pair[last];
                for (int kj = 0; kj < k; kj++) {
                    double bv = Double.POSITIVE_INFINITY;
                    int arg = 0;
                    for (int ki = 0; ki < k; ki++) {
                        double v = prev[ki] + row[ki][j][kj];
                        if (v < bv) {
                            bv = v;
                            arg = ki;
                        }
                    }
                    nb[kj] = bv;
                    bk[kj] = arg;
                    if (bv < minc) minc = bv;
                }
            }
            if (minc >= worst) continue; // no completion of this prefix can beat the current top set
            used[j] = true;
            order[depth] = j;
            dfs(depth + 1, j);
            used[j] = false;
        }
    }

    private void addCandidate(double cost, int lastBin) {
        int[] hb = new int[m];
        hb[m - 1] = lastBin;
        for (int d = m - 1; d >= 1; d--) hb[d - 1] = back[d][hb[d]];
        int st = back[0][hb[0]];
        Candidate c = new Candidate(order.clone(), hb, st, cost);
        c.headings = new double[m];
        c.free = new boolean[m];
        c.depart = new double[m];
        for (int i = 0; i < m; i++) c.headings[i] = bins[hb[i]];
        for (int s = 1; s < m; s++) {
            c.free[s] = pairFree[c.order[s - 1]][hb[s - 1]][c.order[s]][hb[s]];
            c.depart[s] = pairDepart[c.order[s - 1]][hb[s - 1]][c.order[s]][hb[s]];
        }
        c.startTangent = bins[st];

        int idx = Collections.binarySearch(top, c, Comparator.comparingDouble(x -> x.dpCost));
        if (idx < 0) idx = -idx - 1;
        top.add(idx, c);
        int cap = Math.max(1, PlannerConstants.TOP_CANDIDATES);
        if (top.size() > cap) top.remove(top.size() - 1);
        if (top.size() >= cap) worst = top.get(top.size() - 1).dpCost;
    }

    // ------------------------------------------------------------------ stages 2 and 3

    private double evaluate(Candidate c, Pose robotPose, double robotSpeed) {
        List<Piece> pieces = buildPieces(robotPose, c);
        double time = PathTimeModel.coupledTime(pieces, robotSpeed, true);
        double penalty = 0;
        for (Piece p : pieces) penalty += PathTimeModel.outOfFieldPenalty(p);
        c.pieces = pieces;
        c.timeOnly = time;
        return time + penalty;
    }

    /** Coordinate descent over the start direction, every approach heading, and every free departure direction. */
    private void refine(Candidate c, Pose robotPose, double robotSpeed, double binWidth) {
        c.cost = evaluate(c, robotPose, robotSpeed);
        int vars = 1 + m + m; // startTangent, headings[0..m-1], depart[0..m-1] (depart[0] and non-free ones skipped)
        double step = binWidth / 2;
        for (int round = 0; round < PlannerConstants.REFINE_ROUNDS; round++) {
            for (int var = 0; var < vars; var++) {
                if (var > m && !c.free[var - m - 1]) continue;
                if (var == m + 1) continue; // depart[0] is the start tangent
                for (int sign = -1; sign <= 1; sign += 2) {
                    double original = get(c, var);
                    set(c, var, Angle.normalize(original + sign * step));
                    double savedTime = c.timeOnly;
                    List<Piece> savedPieces = c.pieces;
                    double cost = evaluate(c, robotPose, robotSpeed);
                    if (cost < c.cost) {
                        c.cost = cost;
                    } else {
                        set(c, var, original);
                        c.timeOnly = savedTime;
                        c.pieces = savedPieces;
                    }
                }
            }
            step /= 2;
        }
    }

    private double get(Candidate c, int var) {
        if (var == 0) return c.startTangent;
        if (var <= m) return c.headings[var - 1];
        return c.depart[var - m - 1];
    }

    private void set(Candidate c, int var, double value) {
        if (var == 0) c.startTangent = value;
        else if (var <= m) c.headings[var - 1] = value;
        else c.depart[var - m - 1] = value;
    }

    // ------------------------------------------------------------------ Pedro path

    private static Path toPedroPath(List<Piece> pieces) {
        List<Path> parts = new ArrayList<>(pieces.size());
        for (Piece p : pieces) {
            Path part = p.isLine
                    ? Paths.line(p.p0.toPose(), p.p3.toPose())
                    : Paths.curve(p.p0.toPose(), p.p1.toPose(), p.p2.toPose(), p.p3.toPose());
            if (!p.tangentHeading) {
                if (p.holdFraction > 1e-6 && p.holdFraction < 1.0 - 1e-6) {
                    part = part.heading(Interpolator.piecewise()
                            .until(p.holdFraction, Interpolator.constant(p.headingStart))
                            .until(1.0, Interpolator.linear(p.headingStart, p.headingEnd)));
                } else {
                    part = part.linear(p.headingStart, p.headingEnd);
                }
            } else {
                part = PlannerConstants.INTAKE_AT_REAR ? part.reverseTangent() : part.tangent();
            }
            parts.add(part);
        }
        return Paths.path(parts.toArray(new Path[0]));
    }

    // ------------------------------------------------------------------ helpers

    private static List<FieldBall> nearest(List<FieldBall> balls, Vector2D from, int limit) {
        List<FieldBall> sorted = new ArrayList<>(balls);
        Collections.sort(sorted, Comparator.comparingDouble(b -> b.distanceTo(from)));
        if (sorted.size() > limit) return new ArrayList<>(sorted.subList(0, limit));
        return sorted;
    }
}
