package frc.robot.Auton.Stage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.DynamicAutoUtil.DynamicAutonGenerator;
import frc.robot.Auton.DynamicAutoUtil.DynamicPathCommand;
import frc.robot.Auton.DynamicAutoUtil.DynamicAutonGenerator.CompositionType;
import frc.robot.Auton.DynamicAutoUtil.DynamicPathCommand.DecisionPoint;
import frc.robot.Commands.Intake.IntakeAmp;
import frc.robot.Commands.Intake.IntakeShooter;
public class DynamicStageMiddle extends AutonCommandBase{
    private boolean changedPath = false;

    public DynamicStageMiddle(RobotContainer robotContainer){
        super(robotContainer);

        DynamicAutonGenerator auton = new DynamicAutonGenerator();
        auton.addDynamicSection(
            new DynamicPathCommand(robotContainer.getDrivetrain(), 
                Paths.getInstance().THREE_STAGE_MIDDLE, Paths.getInstance().FOUR_STAGE_MIDDLE, false, 
                DecisionPoint.LIMELIGHT.setEndPoint(0.65)
            ).setSecondStop(DecisionPoint.NULL), 
            new SequentialCommandGroup(
                new IntakeShooter()
            ),
            new SequentialCommandGroup(
                new IntakeShooter()
            ),
            CompositionType.PARALLEL_DEADLINE
        );

        this.addCommands(
            new TwoStage(robotContainer),
            auton
        );
    }

    private boolean endCondition(){
        if(!robotContainer.drivetrain.canSeeNote()){
            changedPath=true;
            return true;
        }else{
            return false;
        }
    }
}
