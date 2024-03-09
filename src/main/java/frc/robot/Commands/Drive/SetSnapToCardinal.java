package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveCardinal;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class SetSnapToCardinal extends Command{
    private Drivetrain drivetrain;
    private SwerveCardinal cardinal;

    public SetSnapToCardinal(SwerveCardinal cardinal){
        this.drivetrain = TunerConstants.getInstance().DriveTrain;
        this.cardinal = cardinal;
    }

    @Override
    public void initialize() {
        double redBlue = RobotContainer.getInstance().getSide() == SideMode.RED ? 1.0 : -1.0;
        drivetrain.startSnap(Math.copySign(cardinal.degrees, redBlue));
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
