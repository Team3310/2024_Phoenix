package frc.robot.Auton.Middle.Center.over;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class CenterOverEnd extends AutonCommandBase{

    protected CenterOverEnd(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().CS_CENTER, false).until(()->robotContainer.shooter.hasNote()),
            AimAndShoot(robotContainer)
        );
    }
    
}
