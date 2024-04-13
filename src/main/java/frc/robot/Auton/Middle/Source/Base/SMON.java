package frc.robot.Auton.Middle.Source.Base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class SMON extends AutonCommandBase{
    public SMON(RobotContainer container){
        super(container);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_ON),
            GoToShoot(container, Paths.getInstance().ON_SM)
        );
    }
}
