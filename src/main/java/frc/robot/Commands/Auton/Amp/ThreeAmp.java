package frc.robot.Commands.Auton.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Intake.FullIntakeGo;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.StopAllIntakes;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;

public class ThreeAmp extends AutonCommandBase{
    public ThreeAmp(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new TwoAmp(robotContainer),
            new ParallelDeadlineGroup(
                follow("3Amp"),
                new IntakeAuton()
            ),
            new AimLiftWithOdometryAuton().withTimeout(0.15),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}
