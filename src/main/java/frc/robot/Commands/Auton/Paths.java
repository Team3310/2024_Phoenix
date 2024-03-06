package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

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

    private static Paths instance;

    public static Paths getInstance(){
        if(instance==null){
            instance = new Paths(SideMode.RED);
        }
        return instance;
    }

    private Paths(SideMode side){
        if(side == SideMode.RED){
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
        }else{
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
        }
    }

    public void flip(SideMode sideMode) {
        instance = new Paths(sideMode);
    }
}
