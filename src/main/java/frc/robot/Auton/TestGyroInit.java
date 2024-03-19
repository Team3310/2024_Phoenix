package frc.robot.Auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class TestGyroInit extends AutonCommandBase{
    public TestGyroInit(RobotContainer container){
        super(container);

        SmartDashboard.putNumber("pre gyro angle", container.drivetrain.getBotAz_FieldRelative());
        resetRobotPose(Paths.getInstance().TWO_STAGE_LEFT_PRE_GRAB);
        SmartDashboard.putNumber("post gyro angle", container.drivetrain.getBotAz_FieldRelative());

        this.addCommands(
            Follow(Paths.getInstance().TWO_STAGE_LEFT_PRE_GRAB),
            new InstantCommand(()->SmartDashboard.putNumber("post path gyro angle", container.drivetrain.getBotAz_FieldRelative()))
        );
    }
}
