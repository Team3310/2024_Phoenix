package frc.robot.util;

import frc.robot.Constants;
import frc.robot.util.Choosers.AutonomousChooser.AutonomousMode;
import frc.robot.util.Interpolable.InterpolatingDouble;

public class test {
    public static void main(String[] args) {
        // System.out.println(AutonomousMode.values().length);
        System.out.println(Constants.kLiftAngleMap.getInterpolated(new InterpolatingDouble(Math.hypot(5.02, 1.73))).value);
        // System.out.println(Constants.kRightShooterMap.getInterpolated(new InterpolatingDouble(10.0)).value);
        // System.out.println(Constants.kLeftShooterMap.getInterpolated(new InterpolatingDouble(10.0)).value);
    }
}
