package frc.robot.Commands.Drive;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class WaitUntilAtXBoundary extends Command{
    private final double xBoundary;
    private final Drivetrain drive;

    public WaitUntilAtXBoundary(double xBoundary, RobotContainer container){
        this.drive = container.drivetrain;
        
        if(drive.getSideMode()==SideMode.RED){
            this.xBoundary = GeometryUtil.flipFieldPosition(new Translation2d(xBoundary, xBoundary)).getX();
        }else{
            this.xBoundary = xBoundary;
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return drive.getSideMode()==SideMode.BLUE?drive.getPose().getX()>xBoundary:drive.getPose().getX()<xBoundary;
    }

    @Override
    public void end(boolean interrupted){

    }
}
