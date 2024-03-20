package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Camera.Targeting;
import frc.robot.util.Camera.Targeting.Target;
import frc.robot.util.Camera.Targeting.TargetSimple;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class SetTarget extends Command{
    private TargetSimple targetSimple;
    private Target target;
    private Drivetrain drivetrain;

    public SetTarget(TargetSimple targetSimple){
        this.targetSimple = targetSimple;
        this.drivetrain = TunerConstants.DriveTrain;
        if(drivetrain.getSideMode() == SideMode.BLUE){
            switch (targetSimple){
                case SPEAKER:
                    this.target = target.BLUESPEAKER;
                    break;
                case CENTERPASS:
                    this.target = target.BLUECENTERPASS;
                    break;
                case CORNERPASS:
                    this.target = target.BLUECORNERPASS;
                    break;
            }
        } else {
            switch (targetSimple) {
                case SPEAKER:
                    this.target = target.REDSPEAKER;
                    break;
                case CENTERPASS:
                    this.target = target.REDCENTERPASS;
                    break;
                case CORNERPASS:
                    this.target = target.REDCORNERPASS;
                    break;
            }
        }
    }

    @Override
    public void initialize() {
        Targeting.setTarget(target);
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
