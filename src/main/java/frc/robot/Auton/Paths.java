package frc.robot.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class Paths {
    public final PathPlannerPath TWO_STAGE;
    public final PathPlannerPath THREE_STAGE;
    public final PathPlannerPath FOUR_STAGE;

    public final PathPlannerPath THREE_STAGE_MIDDLE;
    public final PathPlannerPath FOUR_STAGE_MIDDLE;

    public final PathPlannerPath TWO_AMP;
    public final PathPlannerPath TWO_AMP_SOLO;
    public final PathPlannerPath TWO_AMP_SOLO2;
    public final PathPlannerPath THREE_AMP;
    public final PathPlannerPath THREE_AMP_SOLO;
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
    public final PathPlannerPath DRIVE_TEST_TWO;

    public final PathPlannerPath MIDDLE_AN;
    public final PathPlannerPath MIDDLE_MN;
    public final PathPlannerPath MIDDLE_ON;
    public final PathPlannerPath MIDDLE_MS;
    public final PathPlannerPath MS_MIDDLE;
    public final PathPlannerPath CENTER_M;

    public final PathPlannerPath SOURCE_ON;
    public final PathPlannerPath SOURCE_IN;
    public final PathPlannerPath SM_ON;
    public final PathPlannerPath SM_ON3;
    public final PathPlannerPath SM_IN;
    public final PathPlannerPath IN_SM;
    public final PathPlannerPath ON_SM;
    public final PathPlannerPath ON_SM3;
    public final PathPlannerPath IN_SM3;
    public final PathPlannerPath SOURCE_M;
    public final PathPlannerPath STAGE_SM;

    public final PathPlannerPath AMP_IN;
    public final PathPlannerPath AMP_ON;
    public final PathPlannerPath AMP_AM;
    public final PathPlannerPath AMP_M;

    public final PathPlannerPath CLOSE_STAGE_END;

    public final PathPlannerPath SM_CO;
    public final PathPlannerPath CO_SM;

    public final PathPlannerPath SM_GRAB_CLOSE;


    private static Paths instance;

    public static Paths getInstance() {
        if (instance == null) {
            instance = new Paths(SideMode.RED);
        }
        return instance;
    }

    private Paths(SideMode side) {
        // SmartDashboard.putString("Alliance from DS", DriverStation.getAlliance().get().name());
        if (side == SideMode.RED) {
            TWO_STAGE = PathPlannerPath.fromPathFile("2Stage").flipPath();
            THREE_STAGE = PathPlannerPath.fromPathFile("3Stage").flipPath();
            FOUR_STAGE = PathPlannerPath.fromPathFile("4Stage").flipPath();

            THREE_STAGE_MIDDLE = PathPlannerPath.fromPathFile("3StageMiddle").flipPath();
            FOUR_STAGE_MIDDLE = PathPlannerPath.fromPathFile("4StageMiddle").flipPath();

            TWO_AMP = PathPlannerPath.fromPathFile("2Amp").flipPath();
            TWO_AMP_SOLO = PathPlannerPath.fromPathFile("2AmpSolo").flipPath();
            TWO_AMP_SOLO2 = PathPlannerPath.fromPathFile("2AmpSolo2").flipPath();
            THREE_AMP = PathPlannerPath.fromPathFile("3Amp").flipPath();
            THREE_AMP_SOLO = PathPlannerPath.fromPathFile("3AmpSolo").flipPath();
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
            DRIVE_TEST_TWO = PathPlannerPath.fromPathFile("test2").flipPath();

            MIDDLE_AN = PathPlannerPath.fromPathFile("MiddleToAN").flipPath();
            MIDDLE_MN = PathPlannerPath.fromPathFile("MiddleToMN").flipPath();
            MIDDLE_ON = PathPlannerPath.fromPathFile("MiddleToON").flipPath();
            MIDDLE_MS = PathPlannerPath.fromPathFile("MiddleToMS").flipPath();
            MS_MIDDLE = PathPlannerPath.fromPathFile("MSToMiddle").flipPath();
            CENTER_M = PathPlannerPath.fromPathFile("ToMiddleCenter").flipPath();

            SOURCE_ON = PathPlannerPath.fromPathFile("ToSourceON").flipPath();
            SOURCE_IN = PathPlannerPath.fromPathFile("ToSourceIN").flipPath();
            SM_ON = PathPlannerPath.fromPathFile("SourceToON").flipPath();
            SM_ON3 = PathPlannerPath.fromPathFile("SourceToON3").flipPath();
            SM_IN = PathPlannerPath.fromPathFile("SourceToIN").flipPath();
            IN_SM = PathPlannerPath.fromPathFile("IN_SM").flipPath();
            ON_SM = PathPlannerPath.fromPathFile("ON_SM").flipPath();
            IN_SM3 = PathPlannerPath.fromPathFile("IN_SM3").flipPath();
            ON_SM3 = PathPlannerPath.fromPathFile("ON_SM3").flipPath();
            SOURCE_M = PathPlannerPath.fromPathFile("ToMiddleStage").flipPath();
            STAGE_SM = PathPlannerPath.fromPathFile("ToFarMiddleStage").flipPath();

            AMP_IN = PathPlannerPath.fromPathFile("AmpToIN").flipPath();
            AMP_ON = PathPlannerPath.fromPathFile("AmpToON").flipPath();
            AMP_AM = PathPlannerPath.fromPathFile("ToAmpMiddleAmp").flipPath();
            AMP_M = PathPlannerPath.fromPathFile("ToMiddleAmp").flipPath();

            CLOSE_STAGE_END = PathPlannerPath.fromPathFile("CloseStageEnd").flipPath(); 

            SM_CO = PathPlannerPath.fromPathFile("SM_CO").flipPath();
            CO_SM = PathPlannerPath.fromPathFile("CO_SM").flipPath();

            SM_GRAB_CLOSE = PathPlannerPath.fromPathFile("SMGrabClose").flipPath();
        } else {
            TWO_STAGE = PathPlannerPath.fromPathFile("2Stage");
            THREE_STAGE = PathPlannerPath.fromPathFile("3Stage");
            FOUR_STAGE = PathPlannerPath.fromPathFile("4Stage");

            THREE_STAGE_MIDDLE = PathPlannerPath.fromPathFile("3StageMiddle");
            FOUR_STAGE_MIDDLE = PathPlannerPath.fromPathFile("4StageMiddle");

            TWO_AMP = PathPlannerPath.fromPathFile("2Amp");
            TWO_AMP_SOLO = PathPlannerPath.fromPathFile("2AmpSolo");
            TWO_AMP_SOLO2 = PathPlannerPath.fromPathFile("2AmpSolo2");
            THREE_AMP = PathPlannerPath.fromPathFile("3Amp");
            THREE_AMP_SOLO = PathPlannerPath.fromPathFile("3AmpSolo");
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
            DRIVE_TEST_TWO = PathPlannerPath.fromPathFile("test2");

            MIDDLE_AN = PathPlannerPath.fromPathFile("MiddleToAN");
            MIDDLE_MN = PathPlannerPath.fromPathFile("MiddleToMN");
            MIDDLE_ON = PathPlannerPath.fromPathFile("MiddleToON");
            MIDDLE_MS = PathPlannerPath.fromPathFile("MiddleToMS");
            MS_MIDDLE = PathPlannerPath.fromPathFile("MSToMiddle");
            CENTER_M = PathPlannerPath.fromPathFile("ToMiddleCenter");

            SOURCE_ON = PathPlannerPath.fromPathFile("ToSourceON");
            SOURCE_IN = PathPlannerPath.fromPathFile("ToSourceIN");
            SM_ON = PathPlannerPath.fromPathFile("SourceToON");
            SM_ON3 = PathPlannerPath.fromPathFile("SourceToON3");
            SM_IN = PathPlannerPath.fromPathFile("SourceToIN");
            IN_SM = PathPlannerPath.fromPathFile("IN_SM");
            ON_SM = PathPlannerPath.fromPathFile("ON_SM");
            IN_SM3 = PathPlannerPath.fromPathFile("IN_SM3");
            ON_SM3 = PathPlannerPath.fromPathFile("ON_SM3");
            SOURCE_M = PathPlannerPath.fromPathFile("ToMiddleStage");
            STAGE_SM = PathPlannerPath.fromPathFile("ToFarMiddleStage");

            AMP_IN = PathPlannerPath.fromPathFile("AmpToIN");
            AMP_ON = PathPlannerPath.fromPathFile("AmpToON");
            AMP_AM = PathPlannerPath.fromPathFile("ToAmpMiddleAmp");
            AMP_M = PathPlannerPath.fromPathFile("ToMiddleAmp");

            CLOSE_STAGE_END = PathPlannerPath.fromPathFile("CloseStageEnd");

            SM_CO = PathPlannerPath.fromPathFile("SM_CO");
            CO_SM = PathPlannerPath.fromPathFile("CO_SM");

            SM_GRAB_CLOSE = PathPlannerPath.fromPathFile("SMGrabClose");
        }
    }

    public void flip(SideMode sideMode) {
        instance = new Paths(sideMode);
    }
}
