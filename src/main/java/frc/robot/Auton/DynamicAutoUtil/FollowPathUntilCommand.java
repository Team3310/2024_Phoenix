package frc.robot.Auton.DynamicAutoUtil;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;

public class FollowPathUntilCommand extends Command {
    private Drivetrain drive;
    private final PathPlannerPath path;
    private final BooleanSupplier condition;

    public FollowPathUntilCommand(PathPlannerPath path, BooleanSupplier condition) {
        this.path = path;
        this.condition = condition;
        drive = TunerConstants.DriveTrain;
    }

    @Override
    public void initialize() {
        drive.setPath(path, false);
        drive.setDriveMode(DriveMode.AUTON);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return drive.pathDone() || condition.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
    }
}