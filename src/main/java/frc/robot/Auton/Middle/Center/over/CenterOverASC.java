package frc.robot.Auton.Middle.Center.over;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterOverASC extends AutonCommandBase{

    public CenterOverASC(RobotContainer robotContainer) {
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);
        
        this.addCommands(
            new CenterOverStartAmp(robotContainer),
            FollowToIntake(Paths.getInstance().CS_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS),
            FollowToIntake(Paths.getInstance().CS_CCN),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CS),
            new CenterOverEnd(robotContainer)
        );
    }
    
}
