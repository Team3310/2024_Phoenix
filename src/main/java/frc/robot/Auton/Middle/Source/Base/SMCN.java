package frc.robot.Auton.Middle.Source.Base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class SMCN extends AutonCommandBase{
    public SMCN(RobotContainer container){
        super(container);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_CO),
            GoToShoot(container, Paths.getInstance().CO_SM)
        );
    }
}
