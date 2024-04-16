package frc.robot.Auton.Middle.Center;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.base.CenterTwo;

public class CenterFive extends AutonCommandBase{

    public CenterFive(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new CenterTwo(robotContainer),
            FollowToIntake(Paths.getInstance().CN_CCN),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CLOSE, false),
            FollowToIntake(Paths.getInstance().CENTER_AMP),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().CENTER_PODIUM),
            AimAndShoot(robotContainer)
        );
    }
    
}
