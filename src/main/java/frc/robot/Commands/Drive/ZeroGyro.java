package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;

public class SetDriveMode extends Command{
    private Drivetrain drivetrain;
    private DriveMode mode;

    public SetDriveMode(DriveMode mode){
        this.drivetrain = TunerConstants.DriveTrain;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        drivetrain.setDriveMode(mode);
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
