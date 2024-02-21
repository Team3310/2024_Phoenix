package frc.robot.Commands.Flicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Flicker;

public class LoadAmp extends Command{
    private Flicker flicker;

    public LoadAmp(Flicker flicker){
        this.flicker = flicker;

        addRequirements(flicker);
    } 

    @Override
    public void initialize() {
        flicker.setRPM(Constants.AMP_LOAD_RPM);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return !flicker.isNoteLoaded();
    }

    @Override
    public void end(boolean interrupted) {
        flicker.setRPM(0.0);
    }
}
