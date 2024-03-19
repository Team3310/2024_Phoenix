package frc.robot.Commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain;

public class WaitUntilAtPose extends Command{
    private final Translation2d pose;
    private final Drivetrain drive;

    private final double maxDist = 0.2;

    public WaitUntilAtPose(Translation2d pose, RobotContainer container){
        this.pose = pose;
        this.drive = container.drivetrain;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return drive.getPose().getTranslation().getDistance(pose)<=maxDist;
    }

    @Override
    public void end(boolean interrupted){

    }
}
