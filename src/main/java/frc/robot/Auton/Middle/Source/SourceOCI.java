package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Source.Base.SMCN;
import frc.robot.Auton.Middle.Source.Base.SMIN;
import frc.robot.Auton.Middle.Source.Base.SourceON;

public class SourceOCI extends AutonCommandBase{

    public SourceOCI(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new SourceON(robotContainer),
            new SMCN(robotContainer),
            new SMIN(robotContainer)
        );
    }
    
}
