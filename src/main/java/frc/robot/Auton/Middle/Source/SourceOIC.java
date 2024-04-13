package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Source.Base.SMCN;
import frc.robot.Auton.Middle.Source.Base.SMIN;
import frc.robot.Auton.Middle.Source.Base.SourceON;

public class SourceOIC extends AutonCommandBase{

    public SourceOIC(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new SourceON(robotContainer),
            new SMIN(robotContainer),
            new SMCN(robotContainer)
        );
    }
    
}
