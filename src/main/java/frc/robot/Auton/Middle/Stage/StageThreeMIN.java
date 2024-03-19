package frc.robot.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class StageThreeMIN extends AutonCommandBase{
    public StageThreeMIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageTwoIN(robotContainer),
            FollowToIntake(Paths.getInstance().SM_CO),
            Follow(Paths.getInstance().CO_SM),
            AimAndShoot(robotContainer)
        );
    }
}
