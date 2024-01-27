package frc.robot.Commands.Flicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Flicker;

public class FlickerCommand extends Command{
    private Flicker flicker;
    private CommandXboxController joy;


    public FlickerCommand(Flicker flicker, CommandXboxController joy){
        this.flicker = flicker;
        this.joy = joy;

        this.addRequirements(flicker);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        flicker.setPosition(joy.getRightY());
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}
