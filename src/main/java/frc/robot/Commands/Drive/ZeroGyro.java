package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Swerve.TunerConstants;

public class ZeroGyro extends Command{
    private Drivetrain drivetrain;

    public ZeroGyro(){
        this.drivetrain = TunerConstants.DriveTrain;
    }

    @Override
    public void initialize() {
        drivetrain.seedFieldRelative();
        System.out.println("!!!!!!!Gyro Zero!!!!!!!!");
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
