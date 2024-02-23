package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Shooter;

public class Outtake extends Command {
    private Shooter shooter;
    private Flicker flicker;

    public Outtake(Shooter shooter, Flicker flicker) {
        this.shooter = shooter;
        this.flicker = flicker;

        addRequirements(shooter, flicker);
    }

    @Override
    public void initialize() {
        if (shooter.isNoteLoaded()) {
            shooter.setKickerRPM(Constants.KICKER_SCORE_RPM);
        } else if (flicker.isNoteLoaded()) {
            flicker.setRPM(Constants.AMP_SCORE_RPM);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
