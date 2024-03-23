package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeAmpThenToShooter extends SequentialCommandGroup{
    public IntakeAmpThenToShooter(){
        addCommands(
            new IntakeAmp(),
            new IntakeAmpToShooter()
        );
     }
}