package frc.robot.Commands.Auton.Middle.Counter;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Middle.Stage.Base.SMCO;
import frc.robot.Commands.Auton.Middle.Stage.Base.SMON;
import frc.robot.Commands.Auton.Middle.Stage.Base.StageIN;

public class Anti3005Inside extends AutonCommandBase{
    public Anti3005Inside(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageIN(robotContainer),
            new SMCO(robotContainer),
            new SMON(robotContainer)
        );
    }
}
