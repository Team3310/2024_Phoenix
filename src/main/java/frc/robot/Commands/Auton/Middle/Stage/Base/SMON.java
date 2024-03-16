package frc.robot.Commands.Auton.Middle.Stage.Base;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;

public class SMON extends AutonCommandBase{
    public SMON(RobotContainer container){
        super(container);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_ON),
            Follow(Paths.getInstance().ON_SM),
            AimAndShoot(robotContainer)
        );
    }
}
