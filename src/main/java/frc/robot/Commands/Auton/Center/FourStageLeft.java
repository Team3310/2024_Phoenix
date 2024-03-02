package frc.robot.Commands.Auton.Center;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;

public class FourStageLeft extends AutonCommandBase{
    public FourStageLeft(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeStageLeft(robotContainer),
            new ParallelDeadlineGroup(
                follow(Paths.getInstance().FOUR_STAGE_LEFT), 
                new SequentialCommandGroup(
                    new WaitCommand(0.75),
                    new IntakeAuton(false)
                )
            ),
            new AimLiftWithOdometryAuton().until(()->Lift.getInstance().isFinished()),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.3)
        );
    }
}
