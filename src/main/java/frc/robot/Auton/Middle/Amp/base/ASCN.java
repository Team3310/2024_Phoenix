package frc.robot.Auton.Middle.Amp.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class ASCN extends AutonCommandBase{

    public ASCN(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().AS_CN),
            GoToShoot(robotContainer, Paths.getInstance().CN_AS)
        );
    }
    
}
