package frc.robot.Auton.Middle.Center.over;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterOverCAS extends AutonCommandBase{

    public CenterOverCAS(RobotContainer robotContainer) {
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);
        
        this.addCommands(
            new CenterOverStart(robotContainer),
            FollowToIntake(Paths.getInstance().CS_AIN),
            GoToShoot(robotContainer, Paths.getInstance().AIN_CS),
            FollowToIntake(Paths.getInstance().CS_SIN),
            GoToShoot(robotContainer, Paths.getInstance().SIN_CS)
        );
    }
    
}
