package frc.robot.Auton.Middle.Amp.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class AN2AN3 extends AutonCommandBase{

    public AN2AN3(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().AN3_GRAB),
            AimAndShoot(robotContainer)
        );
    }
    
}
