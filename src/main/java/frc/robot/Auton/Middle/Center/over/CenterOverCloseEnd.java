package frc.robot.Auton.Middle.Center.over;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class CenterOverCloseEnd extends AutonCommandBase{

    protected CenterOverCloseEnd(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().CS_CENTER,false,false),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().CENTER_AMP,true,true),
            AimAndShoot(robotContainer),
            FollowToIntakeWaitUntilPose(Paths.getInstance().CENTER_PODIUM, true, true, new Translation2d(2.56, 4.10)),
            AimAndShoot(robotContainer)
        );
    }
    
}
