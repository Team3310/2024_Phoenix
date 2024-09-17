package frc.robot.Auton.Middle.Center.over;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterFiveOverAmp extends AutonCommandBase{

    public CenterFiveOverAmp(RobotContainer robotContainer) {
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);
        
        this.addCommands(
            new CenterOverStartAmp(robotContainer),
            FollowToIntake(Paths.getInstance().CS_CENTER),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().CENTER_AMP),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().CENTER_PODIUM),
            AimAndShoot(robotContainer)
        );
    }
    
}
