package frc.robot.Commands.Auton.Middle.Counter;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Middle.Stage.Base.SMCO;
import frc.robot.Commands.Auton.Middle.Stage.Base.SMIN;
import frc.robot.Commands.Auton.Middle.Stage.Base.StageON;

public class Anti3005 extends AutonCommandBase{
    public Anti3005(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageON(robotContainer),
            new SMCO(robotContainer),
            new SMIN(robotContainer)
        );
    }
}
