package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Source.Base.SMCN;
import frc.robot.Auton.Middle.Source.Base.SMON;
import frc.robot.Auton.Middle.Source.Base.SourceIN;

public class SourceIOC extends AutonCommandBase{

    public SourceIOC(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new SourceIN(robotContainer),
            new SMON(robotContainer),
            new SMCN(robotContainer)
        );
    }
    
}
