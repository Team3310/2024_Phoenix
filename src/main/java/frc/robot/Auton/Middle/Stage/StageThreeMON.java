package frc.robot.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class StageThreeMON extends AutonCommandBase{
    public StageThreeMON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageTwoON(robotContainer),
            FollowToIntake(Paths.getInstance().SM_CO),
            Follow(Paths.getInstance().CO_SM),
            AimAndShoot(robotContainer)
        );
    }
}
