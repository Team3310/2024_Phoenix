package frc.robot.Auton.Middle.Amp.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class ASIN2 extends AutonCommandBase{

    public ASIN2(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().AS_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_AN2, false),
            FollowToIntake(Paths.getInstance().AN2_GRAB),
            AimAndShoot(robotContainer)
        );
    }
    
}
