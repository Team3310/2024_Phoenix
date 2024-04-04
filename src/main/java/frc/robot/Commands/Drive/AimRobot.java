package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;

public class AimRobot extends Command{
    private Drivetrain drivetrain;
    public AimRobot(){
        this.drivetrain = TunerConstants.DriveTrain;
    }

    @Override
    public void initialize() {
        drivetrain.setDriveMode(DriveMode.AIMATTARGET);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished(){
        return !drivetrain.isSnapping();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
