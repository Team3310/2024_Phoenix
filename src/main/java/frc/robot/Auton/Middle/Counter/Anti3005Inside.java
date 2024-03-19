package frc.robot.Auton.Middle.Counter;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Stage.Base.SMCO;
import frc.robot.Auton.Middle.Stage.Base.SMON;
import frc.robot.Auton.Middle.Stage.Base.StageIN;

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
