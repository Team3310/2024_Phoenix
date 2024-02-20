package frc.robot.Commands.Flicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Flicker;

public class SetFlickerRPM extends Command{
    private Flicker flicker;
    private double rpm;

    public SetFlickerRPM(Flicker flicker, double rpm){
        this.flicker = flicker;
        this.rpm = rpm;

        addRequirements(flicker);
    }

    @Override
    public void execute(){

    }

    @Override
    public void initialize(){
        flicker.setRPM(rpm);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}