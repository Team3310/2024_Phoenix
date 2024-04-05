package frc.robot.Auton.Middle.Center.base;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Intake.FullIntakeGo;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;

public class CenterTwo extends AutonCommandBase{
    public CenterTwo(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().C_N);

        this.addCommands(
            new OneAuton(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().C_N), 
                new SetLiftAngle(Lift.getInstance(), 39.5-(1.0+2.0+1.0)),
                new SetLeftShooterRPM(Shooter.getInstance(), 3275.0),
                new SetRightShooterRPM(Shooter.getInstance(), 3400.0),
                new FullIntakeGo()
            ),
            new FeederShootCommandAuton(Shooter.getInstance()).withTimeout(0.025)
        );
    }
}
