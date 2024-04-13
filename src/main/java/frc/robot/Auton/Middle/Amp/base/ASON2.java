package frc.robot.Auton.Middle.Amp.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class ASON2 extends AutonCommandBase{

    public ASON2(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().AS_ON),
            GoToShoot(robotContainer, Paths.getInstance().ON_AN2, false),
            FollowToIntake(Paths.getInstance().AN2_GRAB),
            AimAndShoot(robotContainer)
        );
    }
    
}
