package frc.robot.Auton.Middle.Amp.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class ASIN extends AutonCommandBase{

    public ASIN(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().AS_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_AS)
        );
    }
    
}
