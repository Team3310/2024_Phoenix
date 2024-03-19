package frc.robot.Commands.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Auton.Middle.Stage.Base.StageON;

public class StageThreeON extends AutonCommandBase{
    public StageThreeON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageON(robotContainer),
            FollowToIntake(Paths.getInstance().SM_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_SM3),
            FollowToIntake(Paths.getInstance().SM_GRAB_CLOSE),
            AimAndShoot(robotContainer)
        );
    }
}
