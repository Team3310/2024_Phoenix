// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.Swerve;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.util.Math.Vector2;

import java.util.Arrays;
import org.ejml.simple.SimpleMatrix;

public class SwerveDriveKinematics implements Kinematics<SwerveDriveWheelStates, SwerveDriveWheelPositions> {
   private final SimpleMatrix m_inverseKinematics;
   private final SimpleMatrix m_forwardKinematics;
   private final int m_numModules;
   private final Translation2d[] m_modules;
   private Rotation2d[] m_moduleHeadings;
   private Translation2d m_prevCoR = new Translation2d();

   public SwerveDriveKinematics(Translation2d... moduleTranslationsMeters) {
      if (moduleTranslationsMeters.length < 2) {
         throw new IllegalArgumentException("A swerve drive requires at least two modules");
      } else {
         this.m_numModules = moduleTranslationsMeters.length;
         this.m_modules = (Translation2d[])Arrays.copyOf(moduleTranslationsMeters, this.m_numModules);
         this.m_moduleHeadings = new Rotation2d[this.m_numModules];
         Arrays.fill(this.m_moduleHeadings, new Rotation2d());
         this.m_inverseKinematics = new SimpleMatrix(this.m_numModules * 2, 3);

         for(int i = 0; i < this.m_numModules; ++i) {
            this.m_inverseKinematics.setRow(i * 2 + 0, 0, new double[]{1.0, 0.0, -this.m_modules[i].getY()});
            this.m_inverseKinematics.setRow(i * 2 + 1, 0, new double[]{0.0, 1.0, this.m_modules[i].getX()});
         }

         this.m_forwardKinematics = (SimpleMatrix)this.m_inverseKinematics.pseudoInverse();
         MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
      }
   }

   public void resetHeadings(Rotation2d... moduleHeadings) {
      if (moduleHeadings.length != this.m_numModules) {
         throw new IllegalArgumentException("Number of headings is not consistent with number of module locations provided in constructor");
      } else {
         this.m_moduleHeadings = (Rotation2d[])Arrays.copyOf(moduleHeadings, this.m_numModules);
      }
   }

   public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
      SwerveModuleState[] moduleStates = new SwerveModuleState[this.m_numModules];
      int i;
      if (chassisSpeeds.vxMetersPerSecond == 0.0 && chassisSpeeds.vyMetersPerSecond == 0.0 && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
         for(i = 0; i < this.m_numModules; ++i) {
            moduleStates[i] = new SwerveModuleState(0.0, this.m_moduleHeadings[i]);
         }

         return moduleStates;
      } else {
         if (!centerOfRotationMeters.equals(this.m_prevCoR)) {
            for(i = 0; i < this.m_numModules; ++i) {
               this.m_inverseKinematics.setRow(i * 2 + 0, 0, new double[]{1.0, 0.0, -this.m_modules[i].getY() + centerOfRotationMeters.getY()});
               this.m_inverseKinematics.setRow(i * 2 + 1, 0, new double[]{0.0, 1.0, this.m_modules[i].getX() - centerOfRotationMeters.getX()});
            }

            this.m_prevCoR = centerOfRotationMeters;
         }

         SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
         chassisSpeedsVector.setColumn(0, 0, new double[]{chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond});
         SimpleMatrix moduleStatesMatrix = (SimpleMatrix)this.m_inverseKinematics.mult(chassisSpeedsVector);

         for(i = 0; i < this.m_numModules; ++i) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);
            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);
            moduleStates[i] = new SwerveModuleState(speed, angle);
            this.m_moduleHeadings[i] = angle;
         }

         return moduleStates;
      }
   }

   /**
     * Performs forward kinematics to convert a set of swerve module velocities into a chassis velocity.
     *
     * @param moduleVelocities The velocities of the modules w.r.t the robot.
     * @return The chassis velocity that would result from the module velocities.
     */
    // public ChassisVelocity toChassisVelocity(Vector2... moduleVelocities) {
    //     if (moduleVelocities.length != this.m_numModules) {
    //         throw new IllegalArgumentException("Amount of module velocities given does not match the amount of modules specified in the constructor");
    //     }

    //     SimpleMatrix moduleVelocitiesMatrix = new SimpleMatrix(moduleOffsets.length * 2, 1);
    //     for (int i = 0; i < moduleOffsets.length; i++) {
    //         moduleVelocitiesMatrix.setColumn(0, i * 2,
    //                 moduleVelocities[i].x,
    //                 moduleVelocities[i].y
    //         );
    //     }

    //     SimpleMatrix chassisVelocityVector = forwardKinematics.mult(moduleVelocitiesMatrix);
    //     return new ChassisVelocity(
    //             new Vector2(
    //                     chassisVelocityVector.get(0),
    //                     chassisVelocityVector.get(1)
    //             ),
    //             chassisVelocityVector.get(2)
    //     );
    // }

   public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
      return this.toSwerveModuleStates(chassisSpeeds, new Translation2d());
   }

   public SwerveDriveWheelStates toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
      return new SwerveDriveWheelStates(this.toSwerveModuleStates(chassisSpeeds));
   }

   public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
      if (moduleStates.length != this.m_numModules) {
         throw new IllegalArgumentException("Number of modules is not consistent with number of module locations provided in constructor");
      } else {
         SimpleMatrix moduleStatesMatrix = new SimpleMatrix(this.m_numModules * 2, 1);

         for(int i = 0; i < this.m_numModules; ++i) {
            SwerveModuleState module = moduleStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
         }

         SimpleMatrix chassisSpeedsVector = (SimpleMatrix)this.m_forwardKinematics.mult(moduleStatesMatrix);
         return new ChassisSpeeds(chassisSpeedsVector.get(0, 0), chassisSpeedsVector.get(1, 0), chassisSpeedsVector.get(2, 0));
      }
   }

   public ChassisSpeeds toChassisSpeeds(SwerveDriveWheelStates wheelStates) {
      return this.toChassisSpeeds(wheelStates.states);
   }

   public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {
      if (moduleDeltas.length != this.m_numModules) {
         throw new IllegalArgumentException("Number of modules is not consistent with number of module locations provided in constructor");
      } else {
         SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(this.m_numModules * 2, 1);

         for(int i = 0; i < this.m_numModules; ++i) {
            SwerveModulePosition module = moduleDeltas[i];
            moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
            moduleDeltaMatrix.set(i * 2 + 1, module.distanceMeters * module.angle.getSin());
         }

         SimpleMatrix chassisDeltaVector = (SimpleMatrix)this.m_forwardKinematics.mult(moduleDeltaMatrix);
         return new Twist2d(chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
      }
   }

   public Twist2d toTwist2d(SwerveDriveWheelPositions start, SwerveDriveWheelPositions end) {
      if (start.positions.length != end.positions.length) {
         throw new IllegalArgumentException("Inconsistent number of modules!");
      } else {
         SwerveModulePosition[] newPositions = new SwerveModulePosition[start.positions.length];

         for(int i = 0; i < start.positions.length; ++i) {
            SwerveModulePosition startModule = start.positions[i];
            SwerveModulePosition endModule = end.positions[i];
            newPositions[i] = new SwerveModulePosition(endModule.distanceMeters - startModule.distanceMeters, endModule.angle);
         }

         return this.toTwist2d(newPositions);
      }
   }

   public static void desaturateWheelSpeeds(SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
      double realMaxSpeed = 0.0;
      SwerveModuleState[] var5 = moduleStates;
      int var6 = moduleStates.length;

      int var7;
      SwerveModuleState moduleState;
      for(var7 = 0; var7 < var6; ++var7) {
         moduleState = var5[var7];
         realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
      }

      if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
         var5 = moduleStates;
         var6 = moduleStates.length;

         for(var7 = 0; var7 < var6; ++var7) {
            moduleState = var5[var7];
            moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
         }
      }

   }

   public static void desaturateWheelSpeeds(SwerveModuleState[] moduleStates, Measure<Velocity<Distance>> attainableMaxSpeed) {
      desaturateWheelSpeeds(moduleStates, attainableMaxSpeed.in(Units.MetersPerSecond));
   }

   public static void desaturateWheelSpeeds(SwerveModuleState[] moduleStates, ChassisSpeeds desiredChassisSpeed, double attainableMaxModuleSpeedMetersPerSecond, double attainableMaxTranslationalSpeedMetersPerSecond, double attainableMaxRotationalVelocityRadiansPerSecond) {
      double realMaxSpeed = 0.0;
      SwerveModuleState[] var10 = moduleStates;
      int var11 = moduleStates.length;

      for(int var12 = 0; var12 < var11; ++var12) {
         SwerveModuleState moduleState = var10[var12];
         realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
      }

      if (attainableMaxTranslationalSpeedMetersPerSecond != 0.0 && attainableMaxRotationalVelocityRadiansPerSecond != 0.0 && realMaxSpeed != 0.0) {
         double translationalK = Math.hypot(desiredChassisSpeed.vxMetersPerSecond, desiredChassisSpeed.vyMetersPerSecond) / attainableMaxTranslationalSpeedMetersPerSecond;
         double rotationalK = Math.abs(desiredChassisSpeed.omegaRadiansPerSecond) / attainableMaxRotationalVelocityRadiansPerSecond;
         double k = Math.max(translationalK, rotationalK);
         double scale = Math.min(k * attainableMaxModuleSpeedMetersPerSecond / realMaxSpeed, 1.0);
         SwerveModuleState[] var18 = moduleStates;
         int var19 = moduleStates.length;

         for(int var20 = 0; var20 < var19; ++var20) {
            SwerveModuleState moduleState = var18[var20];
            moduleState.speedMetersPerSecond *= scale;
         }

      }
   }

   public static void desaturateWheelSpeeds(SwerveModuleState[] moduleStates, ChassisSpeeds desiredChassisSpeed, Measure<Velocity<Distance>> attainableMaxModuleSpeed, Measure<Velocity<Distance>> attainableMaxTranslationalSpeed, Measure<Velocity<Angle>> attainableMaxRotationalVelocity) {
      desaturateWheelSpeeds(moduleStates, desiredChassisSpeed, attainableMaxModuleSpeed.in(Units.MetersPerSecond), attainableMaxTranslationalSpeed.in(Units.MetersPerSecond), attainableMaxRotationalVelocity.in(Units.RadiansPerSecond));
   }
}