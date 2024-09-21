package frc.robot.Auton.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Intake.IntakeShooter;

public class FourAmp extends AutonCommandBase{
    public FourAmp(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeAmp(robotContainer),
            //TODO replace with new method
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().FOUR_AMP),
                new IntakeShooter()
            ),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().SNEAK5),
            GoToShoot(robotContainer, Paths.getInstance().CCN_CS)
        );
    }
}
