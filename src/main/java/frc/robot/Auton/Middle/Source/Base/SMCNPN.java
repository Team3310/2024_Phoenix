package frc.robot.Auton.Middle.Source.Base;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;

public class SMCNPN extends AutonCommandBase{

    public SMCNPN(RobotContainer robotContainer, CNPNPath path) {
        super(robotContainer);
        
        this.addCommands(
            FollowToIntake(Paths.getInstance().SM_CO),
            GoToShoot(robotContainer, path.getPath(), false),
            FollowToIntake(Paths.getInstance().SM_GRAB_CLOSE),
            AimAndShoot(robotContainer)
        );
    }
    
    public enum CNPNPath{
        CN(Paths.getInstance().CN_SM3),
        CCNS(Paths.getInstance().CCNS_SM3),
        ;

        private final PathPlannerPath path;

        private CNPNPath(PathPlannerPath path){
            this.path = path;
        }

        public PathPlannerPath getPath(){
            return this.path;
        }
    }
}
