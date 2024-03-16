package frc.robot.Commands.Auton.Middle.Stage.Base;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;

public class SMCO extends AutonCommandBase{
    public SMCO(RobotContainer container){
        super(container);

        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_CO),
            Follow(Paths.getInstance().CO_SM),
            AimAndShoot(robotContainer)
        );
    }
}
