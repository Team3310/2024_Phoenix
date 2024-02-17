package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class SetClimberInches extends Command{
    private Climber climber;
    private double inches;


    public SetClimberInches(Climber climber, double inches){
        this.climber = climber;
        this.inches = inches;

        addRequirements(climber);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        climber.setPosition(inches);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}
