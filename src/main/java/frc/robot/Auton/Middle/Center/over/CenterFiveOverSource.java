package frc.robot.Auton.Middle.Center.over;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterFiveOverSource extends AutonCommandBase{

    public CenterFiveOverSource(RobotContainer robotContainer) {
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);
        
        this.addCommands(
            new CenterOverStartSource(robotContainer),
            new CenterOverCloseEnd(robotContainer)
        );
    }
    
}
