package frc.robot.Auton.DynamicAutoUtil;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Control.TimeDelayedBoolean;

public class FollowPathUntilCommandWithDelay extends Command {
    private Drivetrain drive;
    private final PathPlannerPath path;
    private final BooleanSupplier supplier;
    private final double timeout;
    private final TimeDelayedBoolean condition = new TimeDelayedBoolean();

    public FollowPathUntilCommandWithDelay(PathPlannerPath path, BooleanSupplier supplier, double timeout) {
        this.path = path;
        this.supplier = supplier;
        this.timeout = timeout;
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
        return drive.pathDone() || condition.update(supplier.getAsBoolean(), timeout);
    }

    @Override
    public void end(boolean interrupted) {
    }
}