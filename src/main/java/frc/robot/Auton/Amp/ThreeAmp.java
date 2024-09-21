package frc.robot.Auton.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Intake.IntakeShooter;

public class ThreeAmp extends AutonCommandBase{
    public ThreeAmp(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new TwoAmp(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().THREE_AMP),
                new IntakeShooter()
            ),
            AimAndShoot(robotContainer)
        );
    }
}
