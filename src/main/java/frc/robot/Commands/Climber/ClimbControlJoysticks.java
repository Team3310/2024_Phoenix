package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;

public class ClimbControlJoysticks extends Command {
    private Climber climber;
    private CommandXboxController controller;

    public ClimbControlJoysticks(Climber climber, CommandXboxController  controller) {
        this.controller = controller;
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        double speed = controller.getLeftY();

        if (Math.abs(speed) > Constants.CLIMBER_MIN_PERCENT_BUS) {
            climber.setLeftSpeed(speed);
            climber.setRightSpeed(speed);
        } else {
            climber.setHoldLeftClimber();
            climber.setHoldRightClimber();
        }
    }

    // @Override
    // public void end(boolean interrupted) {
    //     climber.setHoldLeftClimber();
    //     climber.setHoldRightClimber();
    // }

}
