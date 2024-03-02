package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveOrientation;
import frc.robot.Swerve.TunerConstants;

public class SetDriveOrientation extends Command{
    private Drivetrain drivetrain;
    private DriveOrientation orientation;

    public SetDriveOrientation(DriveOrientation orientation){
        this.drivetrain = TunerConstants.DriveTrain;
        this.orientation = orientation;
    }

    @Override
    public void initialize() {
        drivetrain.setDriveOrientation(orientation);
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
