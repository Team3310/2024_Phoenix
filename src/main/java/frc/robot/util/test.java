package frc.robot.util;

import frc.robot.Constants;
import frc.robot.util.Interpolable.InterpolatingDouble;

public class test {
    public static void main(String[] args) {
        System.out.println(Constants.kLiftAngleMap.getInterpolated(new InterpolatingDouble(10.0)).value);
        System.out.println(Constants.kRightShooterMap.getInterpolated(new InterpolatingDouble(10.0)).value);
        System.out.println(Constants.kLeftShooterMap.getInterpolated(new InterpolatingDouble(10.0)).value);
    }
}
