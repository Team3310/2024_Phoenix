package frc.robot.Commands.Flicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Flicker;

public class ScoreAmp extends Command{
    private Flicker flicker;

    public ScoreAmp(Flicker flicker){
        this.flicker = flicker;

        addRequirements(flicker);
    } 

    @Override
    public void initialize() {
        flicker.setRPM(Constants.AMP_SCORE_RPM);
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
