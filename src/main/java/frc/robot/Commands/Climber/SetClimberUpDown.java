package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;

public class SetClimberUpDown extends Command {
    private Climber climber;

    public SetClimberUpDown(Climber climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (climber.getRightPositionInches() < (Constants.CLIMBER_MIN_INCHES + 2.0)) {
            climber.setPosition(Constants.CLIMBER_MAX_INCHES);
        }
        else if (climber.getRightPositionInches() > (Constants.CLIMBER_MAX_INCHES - 2.0)) {
            climber.setPosition(Constants.CLIMBER_MIN_INCHES);
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
