package frc.robot.Auton.Middle.Center;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Middle.Center.base.CenterTwo;

public class CenterFiveSource extends AutonCommandBase{

    public CenterFiveSource(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new CenterTwo(robotContainer),
            FollowToIntake(Paths.getInstance().CN_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS, false),
            Follow(Paths.getInstance().CS_CENTER),
            FollowToIntake(Paths.getInstance().CENTER_AMP),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().CENTER_PODIUM),
            AimAndShoot(robotContainer)
        );
    }
    
}
