package frc.robot.Commands.Auton.Center;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;

public class ThreeStageLeft extends AutonCommandBase{
    public ThreeStageLeft(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new TwoStageLeft(robotContainer),
            new ParallelDeadlineGroup(
                follow("3StageLeft"), 
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
