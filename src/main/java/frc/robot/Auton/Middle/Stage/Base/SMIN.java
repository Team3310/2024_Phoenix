package frc.robot.Auton.Middle.Stage.Base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class SMIN extends AutonCommandBase{
    public SMIN(RobotContainer container){
        super(container);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_IN),
            GoToShoot(container, Paths.getInstance().IN_SM)
        );
    }
}
