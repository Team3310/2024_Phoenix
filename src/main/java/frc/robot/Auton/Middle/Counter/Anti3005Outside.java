package frc.robot.Auton.Middle.Counter;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Stage.Base.SMCO;
import frc.robot.Auton.Middle.Stage.Base.SMIN;
import frc.robot.Auton.Middle.Stage.Base.StageON;

public class Anti3005Outside extends AutonCommandBase{
    public Anti3005Outside(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new StageON(robotContainer),
            new SMCO(robotContainer),
            new SMIN(robotContainer)
        );
    }
}
