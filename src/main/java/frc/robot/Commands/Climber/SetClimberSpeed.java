package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class SetClimberSpeed extends Command{
    private Climber climber;
    private double speed;


    public SetClimberSpeed(Climber climber, double speed){
        this.climber = climber;
        this.speed = speed;

        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setSpeed(speed, speed);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}
