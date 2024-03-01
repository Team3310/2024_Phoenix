package frc.robot.Commands.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.RobotContainer;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class Paths {
    private final PathPlannerPath TWO_STAGE_PRE_GRAB;
    private final PathPlannerPath TWO_STAGE_GRAB;
    private final PathPlannerPath THREE_STAGE;
    private final PathPlannerPath FOUR_STAGE;

    private final PathPlannerPath THREE_STAGE_MIDDLE;
    private final PathPlannerPath FOUR_STAGE_MIDDLE;

    // private final PathPlannerPath TWO_AMP;
    // private final PathPlannerPath THREE_AMP;
    // private final PathPlannerPath FOUR_AMP;

    // private final PathPlannerPath TWO_STAGE_LEFT;
    // private final PathPlannerPath THREE_STAGE_LEFT;
    // private final PathPlannerPath FOUR_STAGE_LEFT;

    public Paths(){
        TWO_STAGE_PRE_GRAB = PathPlannerPath.fromPathFile("2StagePreGrab");
        TWO_STAGE_GRAB = PathPlannerPath.fromPathFile("2StageGrab");
        THREE_STAGE = PathPlannerPath.fromPathFile("3Stage");
        FOUR_STAGE = PathPlannerPath.fromPathFile("4Stage");
        
        THREE_STAGE_MIDDLE = PathPlannerPath.fromPathFile("3StageMiddle");
        FOUR_STAGE_MIDDLE = PathPlannerPath.fromPathFile("4StageMiddle");
    }
}
