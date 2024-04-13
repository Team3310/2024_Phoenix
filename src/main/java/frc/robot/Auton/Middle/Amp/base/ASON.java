package frc.robot.Auton.Middle.Amp.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class ASON extends AutonCommandBase{

    public ASON(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().AS_ON),
            GoToShoot(robotContainer, Paths.getInstance().ON_AS)
        );
    }
    
}
