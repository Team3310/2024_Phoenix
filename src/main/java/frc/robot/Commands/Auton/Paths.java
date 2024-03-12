package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.GeometryUtil;

import frc.robot.util.Choosers.SideChooser.SideMode;

public class Paths {
    public final PathPlannerPath TWO_STAGE;
    public final PathPlannerPath THREE_STAGE;
    public final PathPlannerPath FOUR_STAGE;

    public final PathPlannerPath THREE_STAGE_MIDDLE;
    public final PathPlannerPath FOUR_STAGE_MIDDLE;

    public final PathPlannerPath TWO_AMP;
    public final PathPlannerPath THREE_AMP;
    public final PathPlannerPath FOUR_AMP;

    public final PathPlannerPath TWO_STAGE_LEFT_PRE_GRAB;
    public final PathPlannerPath TWO_STAGE_LEFT_GRAB;
    public final PathPlannerPath THREE_STAGE_LEFT;
    public final PathPlannerPath FOUR_STAGE_LEFT;

    public final PathPlannerPath THREE_STAGE_COUNTER;

    public final PathPlannerPath THREE_FAST;
    public final PathPlannerPath FOUR_FAST;
    public final PathPlannerPath FAST_END;

    public final PathPlannerPath DRIVE_TEST;

    public final PathPlannerPath MIDDLE_AN;
    public final PathPlannerPath MIDDLE_MN;
    public final PathPlannerPath MIDDLE_ON;
    public final PathPlannerPath MIDDLE_MS;
    public final PathPlannerPath MS_MIDDLE;
    public final PathPlannerPath CENTER_M;

    public final PathPlannerPath SOURCE_ON;
    public final PathPlannerPath SOURCE_IN;
    public final PathPlannerPath SM_SS;
    public final PathPlannerPath SS_SM;
    public final PathPlannerPath SOURCE_M;
    public final PathPlannerPath STAGE_SM;

    public final PathPlannerPath AMP_IN;
    public final PathPlannerPath AMP_ON;
    public final PathPlannerPath AM_AS;
    public final PathPlannerPath AS_AM;
    public final PathPlannerPath AMP_AM;
    public final PathPlannerPath AMP_M;

    private static Paths instance;

    public static Paths getInstance() {
        if (instance == null) {
            instance = new Paths(SideMode.RED);
        }
        return instance;
    }

    private Paths(SideMode side) {
        if (side == SideMode.RED) {
            TWO_STAGE = PathPlannerPath.fromPathFile("2Stage").flipPath();
            THREE_STAGE = PathPlannerPath.fromPathFile("3Stage").flipPath();
            FOUR_STAGE = PathPlannerPath.fromPathFile("4Stage").flipPath();

            THREE_STAGE_MIDDLE = PathPlannerPath.fromPathFile("3StageMiddle").flipPath();
            FOUR_STAGE_MIDDLE = PathPlannerPath.fromPathFile("4StageMiddle").flipPath();

            TWO_AMP = PathPlannerPath.fromPathFile("2Amp").flipPath();
            THREE_AMP = PathPlannerPath.fromPathFile("3Amp").flipPath();
            FOUR_AMP = PathPlannerPath.fromPathFile("4Amp").flipPath();

            TWO_STAGE_LEFT_PRE_GRAB = PathPlannerPath.fromPathFile("2StageLeftPreGrab").flipPath();
            TWO_STAGE_LEFT_GRAB = PathPlannerPath.fromPathFile("2StageLeftGrab").flipPath();
            THREE_STAGE_LEFT = PathPlannerPath.fromPathFile("3StageLeft").flipPath();
            FOUR_STAGE_LEFT = PathPlannerPath.fromPathFile("4StageLeft").flipPath();

            THREE_STAGE_COUNTER = PathPlannerPath.fromPathFile("3StageLeftCounter").flipPath();

            THREE_FAST = PathPlannerPath.fromPathFile("Fast3MiddleStage").flipPath();
            FOUR_FAST = PathPlannerPath.fromPathFile("Fast4MiddleStage").flipPath();
            FAST_END = PathPlannerPath.fromPathFile("lastFastMiddle").flipPath();

            DRIVE_TEST = PathPlannerPath.fromPathFile("DriveTest").flipPath();

            MIDDLE_AN = PathPlannerPath.fromPathFile("MiddleToAN").flipPath();
            MIDDLE_MN = PathPlannerPath.fromPathFile("MiddleToMN").flipPath();
            MIDDLE_ON = PathPlannerPath.fromPathFile("MiddleToON").flipPath();
            MIDDLE_MS = PathPlannerPath.fromPathFile("MiddleToMS").flipPath();
            MS_MIDDLE = PathPlannerPath.fromPathFile("MSToMiddle").flipPath();
            CENTER_M = PathPlannerPath.fromPathFile("ToMiddleCenter").flipPath();

            SOURCE_ON = PathPlannerPath.fromPathFile("SourceToON").flipPath();
            SOURCE_IN = PathPlannerPath.fromPathFile("SourceToIN").flipPath();
            SM_SS = PathPlannerPath.fromPathFile("SMToSS").flipPath();
            SS_SM = PathPlannerPath.fromPathFile("SSToSM").flipPath();
            SOURCE_M = PathPlannerPath.fromPathFile("ToMiddleStage").flipPath();
            STAGE_SM = PathPlannerPath.fromPathFile("ToFarMiddleStage").flipPath();

            AMP_IN = PathPlannerPath.fromPathFile("AmpToIN").flipPath();
            AMP_ON = PathPlannerPath.fromPathFile("AmpToON").flipPath();
            AM_AS = PathPlannerPath.fromPathFile("AMToAS").flipPath();
            AS_AM = PathPlannerPath.fromPathFile("ASToAM").flipPath();
            AMP_AM = PathPlannerPath.fromPathFile("ToAmpMiddleAmp").flipPath();
            AMP_M = PathPlannerPath.fromPathFile("ToMiddleAmp").flipPath();
        } else {
            TWO_STAGE = PathPlannerPath.fromPathFile("2Stage");
            THREE_STAGE = PathPlannerPath.fromPathFile("3Stage");
            FOUR_STAGE = PathPlannerPath.fromPathFile("4Stage");

            THREE_STAGE_MIDDLE = PathPlannerPath.fromPathFile("3StageMiddle");
            FOUR_STAGE_MIDDLE = PathPlannerPath.fromPathFile("4StageMiddle");

            TWO_AMP = PathPlannerPath.fromPathFile("2Amp");
            THREE_AMP = PathPlannerPath.fromPathFile("3Amp");
            FOUR_AMP = PathPlannerPath.fromPathFile("4Amp");

            TWO_STAGE_LEFT_PRE_GRAB = PathPlannerPath.fromPathFile("2StageLeftPreGrab");
            TWO_STAGE_LEFT_GRAB = PathPlannerPath.fromPathFile("2StageLeftGrab");
            THREE_STAGE_LEFT = PathPlannerPath.fromPathFile("3StageLeft");
            FOUR_STAGE_LEFT = PathPlannerPath.fromPathFile("4StageLeft");

            THREE_STAGE_COUNTER = PathPlannerPath.fromPathFile("3StageLeftCounter");

            THREE_FAST = PathPlannerPath.fromPathFile("Fast3MiddleStage");
            FOUR_FAST = PathPlannerPath.fromPathFile("Fast4MiddleStage");
            FAST_END = PathPlannerPath.fromPathFile("lastFastMiddle");

            DRIVE_TEST = PathPlannerPath.fromPathFile("DriveTest");

            MIDDLE_AN = PathPlannerPath.fromPathFile("MiddleToAN");
            MIDDLE_MN = PathPlannerPath.fromPathFile("MiddleToMN");
            MIDDLE_ON = PathPlannerPath.fromPathFile("MiddleToON");
            MIDDLE_MS = PathPlannerPath.fromPathFile("MiddleToMS");
            MS_MIDDLE = PathPlannerPath.fromPathFile("MSToMiddle");
            CENTER_M = PathPlannerPath.fromPathFile("ToMiddleCenter");

            SOURCE_ON = PathPlannerPath.fromPathFile("SourceToON");
            SOURCE_IN = PathPlannerPath.fromPathFile("SourceToIN");
            SM_SS = PathPlannerPath.fromPathFile("SMToSS");
            SS_SM = PathPlannerPath.fromPathFile("SSToSM");
            SOURCE_M = PathPlannerPath.fromPathFile("ToMiddleStage");
            STAGE_SM = PathPlannerPath.fromPathFile("ToFarMiddleStage");

            AMP_IN = PathPlannerPath.fromPathFile("AmpToIN");
            AMP_ON = PathPlannerPath.fromPathFile("AmpToON");
            AM_AS = PathPlannerPath.fromPathFile("AMToAS");
            AS_AM = PathPlannerPath.fromPathFile("ASToAM");
            AMP_AM = PathPlannerPath.fromPathFile("ToAmpMiddleAmp");
            AMP_M = PathPlannerPath.fromPathFile("ToMiddleAmp");
        }
    }

    public void flip(SideMode sideMode) {
        instance = new Paths(sideMode);
    }
}
