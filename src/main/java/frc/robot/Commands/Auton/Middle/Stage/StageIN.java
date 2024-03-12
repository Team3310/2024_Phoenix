package frc.robot.Commands.Auton.Middle.Stage;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.OneAuton;
import frc.robot.Commands.Auton.Paths;

public class StageIN extends AutonCommandBase{
    public StageIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new OneAuton(robotContainer, Paths.getInstance().STAGE_SM),
            Follow(Paths.getInstance().STAGE_SM),
            FollowToIntake(Paths.getInstance().SOURCE_IN),
            AimAndShoot(robotContainer)
        );
    }
}
