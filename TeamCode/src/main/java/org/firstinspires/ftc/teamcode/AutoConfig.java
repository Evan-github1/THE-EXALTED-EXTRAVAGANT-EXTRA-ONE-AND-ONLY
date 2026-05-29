package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotWidth;

public class AutoConfig {

//    /**
//     * Set by any autonomous OpMode before it ends.
//     * TeleOp reads this to continue from where auto left off.
//     * Null if no auto has run this session.
//     */
    public static Pose lastAutoEndPose = null;

    public static boolean isRed = true; // Set by auto, read by teleop to determine alliance color for teleop. Assume red alliance.
    private static Pose RED_OFFSET = new Pose(0,-3,0);


    //-- Blue Alliance Stuff ----
    public static final Pose BLUE_FAR_START = new Pose(46.75, robotLength()/2, Math.toRadians(-90));
    public static final Pose BLUE_BALL1_START     = new Pose(50, 36, -Math.PI);
    public static final Pose BLUE_BALL1_END       = new Pose(13, 36, -Math.PI);
    public static final Pose BLUE_BALL2_START     = new Pose(50, 60, -Math.PI);
    public static final Pose BLUE_BALL2_END       = new Pose(13, 60, -Math.PI);
    public static final Pose BLUE_BALL3_START     = new Pose(50, 84, -Math.PI);
    public static final Pose BLUE_BALL3_END       = new Pose(13, 84, -Math.PI);
    public static final Pose BLUE_FAR_SCORE       = new Pose(53,16,Math.atan2(16-141.5,53-0));
    public static final Pose BLUE_CLEAR = new Pose(14,72,-Math.PI);
    public static final Pose BLUE_READY_CLEAR = new Pose(42,72,-Math.PI);

    // ── SHARED RED (UhUhREDFAR + DoomREDFAR) ─────────────────────
    public static final Pose RED_FAR_START     =      BLUE_FAR_START.mirror();
    public static final Pose RED_FAR_SCORE           = BLUE_FAR_SCORE.mirror();
    public static final Pose RED_BALL1_START     = BLUE_BALL1_START.mirror().plus(RED_OFFSET);
    public static final Pose RED_BALL1_END       = BLUE_BALL1_END.mirror().plus(RED_OFFSET);
    public static final Pose RED_BALL2_START     = BLUE_BALL2_START.mirror().plus(RED_OFFSET);
    public static final Pose RED_BALL2_END       = BLUE_BALL2_END.mirror().plus(RED_OFFSET);
    public static final Pose RED_BALL3_START     = BLUE_BALL3_START.mirror().plus(RED_OFFSET);
    public static final Pose RED_BALL3_END       = BLUE_BALL3_END.mirror().plus(RED_OFFSET);
    public static final Pose RED_CLEAR = BLUE_CLEAR.mirror().plus(RED_OFFSET);
    public static final Pose RED_READY_CLEAR = BLUE_READY_CLEAR.mirror().plus(RED_OFFSET);
    public static final Pose RED_CLOSE_SCORE     = new Pose(85, 85, Math.PI + Math.PI/4);
    public static final Pose RED_PARK = new Pose(40.7,32,-Math.PI);


    // ── UHUH RED FAR only ─────────────────────────────────────────────
    public static final Pose RED_FAR_CLEAR2 = new Pose(118, 67, 0);
    public static final Pose RED_FAR_CORNER1         = new Pose(141.5 - robotWidth()/2 - 5, 40 + robotLength()/2, Math.toRadians(-90));
    public static final Pose RED_FAR_CORNER2         = new Pose(141.5 - robotWidth()/2, 2 + robotLength()/2 + 3, Math.toRadians(-90));

    public static final Pose RED_FAR_CORNER_DIRECT   = new Pose(141.5-robotWidth()/2,robotLength()/2,Math.toRadians(0));

    // ── DOOM RED FAR only ─────────────────────────────────────────────
    public static final Pose DOOM_RED_FAR_CLEAR_READY = new Pose(100, 67, 0);

    // ── SHARED BLUE (UhUhBLUEFAR + FarOnlyBLUE + DoomBLUEFAR) ───
    // Now derived as mirrors of RED FAR poses.
    // Old manual definitions (for reference / rollback):
    // public static final Pose BLUE_FAR_START       = new Pose(141.5 - 96, robotLength()/2, Math.PI - Math.toRadians(-90));
    // public static final Pose BLUE_FAR_SCORE       = new Pose(141.5 - 88.6, 11.7, Math.PI - Math.toRadians(-115.5));
    // public static final Pose BLUE_BALL1_START = new Pose(141.5 - 95, 36, Math.PI);
    // public static final Pose BLUE_BALL1_END   = new Pose(141.5 - 130, 36, Math.PI);
    // public static final Pose BLUE_BALL2_START = new Pose(141.5 - 95, 60, Math.PI);
    // public static final Pose BLUE_BALL2_END   = new Pose(141.5 - 130, 60, Math.PI);
    // public static final Pose BLUE_BALL3_START = new Pose(141.5 - 95, 84, Math.PI);
    // public static final Pose BLUE_BALL3_END   = new Pose(141.5 - 125, 84, Math.PI);
    // public static final Pose BLUE_FAR_CLEAR       = new Pose(141.5 - 125, 72, Math.PI);
    // public static final Pose BLUE_CLOSE_SCORE = new Pose(141.5 - 85, 85, -Math.PI/4);
    public static final Pose BLUE_CLOSE_SCORE    = RED_CLOSE_SCORE.mirror().minus(RED_OFFSET);
    public static final Pose BLUE_PARK = RED_PARK.mirror().minus(RED_OFFSET);

    // ── UHUH BLUE FAR only ────────────────────────────────────────────
    // Old manual definitions:
    // public static final Pose BLUE_FAR_CLEAR2      = new Pose(141.5 - 115, 72, Math.PI);
    // public static final Pose BLUE_FAR_CORNER1     = new Pose(robotWidth()/2, 30 + robotLength()/2, Math.PI - Math.toRadians(-90));
    // public static final Pose BLUE_FAR_CORNER2     = new Pose(robotWidth()/2, 2 + robotLength()/2 + 3, Math.PI - Math.toRadians(-90));
    public static final Pose BLUE_FAR_CLEAR2         = RED_FAR_CLEAR2.mirror().minus(RED_OFFSET);
    public static final Pose BLUE_FAR_CORNER1        = RED_FAR_CORNER1.mirror().minus(RED_OFFSET);
    public static final Pose BLUE_FAR_CORNER2        = RED_FAR_CORNER2.mirror().minus(RED_OFFSET);
    public static final Pose BLUE_FAR_CORNER_DIRECT  = RED_FAR_CORNER_DIRECT.mirror().minus(RED_OFFSET);
    // ── DOOM BLUE CLOSE (DoomAndDisgust_PEDROBLUECLOSE) ───────────────
    public static final Pose DOOM_BLUE_CLOSE_START    = new Pose(48, 144 - robotLength()/2, Math.PI/2);
    public static final Pose DOOM_BLUE_CLOSE_SCORE    = new Pose(60, 144 - 60, -Math.PI/4);
    public static final Pose DOOM_BLUE_CLOSE_LEAVE    = new Pose(12, 144 - 48, 0);
    // ── DOOM RED CLOSE (DoomAndDisgust_PEDROREDCLOSE) ─────────────────
    public static final Pose DOOM_RED_CLOSE_START     = DOOM_BLUE_CLOSE_START.mirror().plus(RED_OFFSET);
    public static final Pose DOOM_RED_CLOSE_SCORE     = DOOM_BLUE_CLOSE_SCORE.mirror().plus(RED_OFFSET);
    public static final Pose DOOM_RED_CLOSE_LEAVE     = DOOM_BLUE_CLOSE_LEAVE.mirror().plus(RED_OFFSET);

    public static final Pose GET_LANDING_ZONE_BLUE1 = new Pose(2+robotWidth()/2, 2+robotLength()/2, Math.PI/2);
    public static final Pose GET_LANDING_ZONE_BLUE2 = new Pose(2+robotWidth()/2,40,Math.PI/2);
    public static final Pose GET_LANDING_ZONE_RED1 = GET_LANDING_ZONE_BLUE1.mirror();
    public static final Pose GET_LANDING_ZONE_RED2 = GET_LANDING_ZONE_BLUE2.mirror();
}
