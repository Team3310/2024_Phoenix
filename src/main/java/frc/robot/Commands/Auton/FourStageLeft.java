package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;

public class FourStageLeft extends AutonCommandBase{
    public FourStageLeft(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeStageLeft(robotContainer),
            new ParallelDeadlineGroup(
                follow("4StageLeft"), 
                new SequentialCommandGroup(
                    new WaitCommand(0.75),
                    new IntakeAuton(false)
                )
            ),
            new AimLiftWithOdometryAuton().withTimeout(0.25),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.3)
        );
    }
}
