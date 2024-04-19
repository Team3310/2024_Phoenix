package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Source.Base.SMCN;
import frc.robot.Auton.Middle.Source.Base.SMON;
import frc.robot.Auton.Middle.Source.Base.SourceIN;

public class SourceICO extends AutonCommandBase{

    public SourceICO(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new SourceIN(robotContainer),
            new SMCN(robotContainer),
            new SMON(robotContainer)
        );
    }
}
