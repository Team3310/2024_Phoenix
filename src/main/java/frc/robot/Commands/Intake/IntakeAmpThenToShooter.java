package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeAmpThenToShooter extends SequentialCommandGroup{
    public IntakeAmpThenToShooter(){
        addCommands(
            new IntakeAmp(),
            new WaitCommand(0.1),
            new IntakeAmpToShooter()
        );
     }
}