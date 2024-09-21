package frc.robot.Auton.Middle.Center.over;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterFiveOver extends AutonCommandBase{

    public CenterFiveOver(RobotContainer robotContainer) {
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);
        
        this.addCommands(
            new CenterOverStartCenter(robotContainer),
            new CenterOverCloseEnd(robotContainer)
        );
    }
    
}
