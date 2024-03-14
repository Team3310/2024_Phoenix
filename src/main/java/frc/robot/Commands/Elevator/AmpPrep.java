package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Flicker.LoadAmp;

public class AmpPrep extends SequentialCommandGroup{
    public AmpPrep(RobotContainer container){
        this.addCommands(
            new SetElevatorInches(container.elevator, Constants.AMP_SCORE_INCHES).alongWith(new LoadAmp(container.flicker))
        );
    }
}
