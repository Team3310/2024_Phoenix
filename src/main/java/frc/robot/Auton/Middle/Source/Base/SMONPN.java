package frc.robot.Auton.Middle.Source.Base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class SMONPN extends AutonCommandBase{
    public SMONPN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_ON),
            GoToShoot(robotContainer, Paths.getInstance().ON_SM3, false),
            FollowToIntake(Paths.getInstance().SM_GRAB_CLOSE),
            AimAndShoot(robotContainer)
        );
    }
}
