package frc.robot.Auton.Middle.Source.Base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class SMINPN extends AutonCommandBase{
    public SMINPN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_SM3, false),
            FollowToIntake(Paths.getInstance().SM_GRAB_CLOSE),
            AimAndShoot(robotContainer)
        );
    }
}
