// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REAL;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public static class LimelightConstants {
    public static final double climbLeftAngle = 45;
    public static final double climbRightAngle = 90;
    public static final double normalAngle = 135;
  }



  public static class SwerveConstants {

    public static final double[] hub_dimensions = new double[] {0.6604*2, 0.6604*2};
    public static final double[] net_dimensions = new double[] {0.492252*2, 0.805307*2};

    // Gear Ratio
    public static final double driveReduction = 5.36;
    public static final double steerReduction = 18.75;

    /* Steer Motor PID Values */

    public static final double angleKP = 50;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKS = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.25; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.1996; // TODO: This must be tuned to specific robot
    public static final double driveKV = 0.6396;
    public static final double driveKA = 0.1;
    //public static final double driveKA = 0.3;

    public static final Slot0Configs intrinsicsD =
        new Slot0Configs().withKP(driveKP).withKD(driveKD).withKV(driveKV).withKS(driveKS).withKA(driveKA);
    public static final Slot0Configs instrinsicsS =
        new Slot0Configs().withKP(angleKP).withKD(angleKD).withKS(angleKS);

    public record SwerveModuleConstants(
        int driveMotorID,
        int angleMotorID,
        int canCoderID,
        double angleOffset,
        boolean invertEncoder,
        boolean invertDrive,
        boolean invertSteer) {}

    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 3;
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = 0.1113;
      // 36.123046875 + 2.28515625)
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 6;
      public static final boolean invertDrive = true;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;

      public static final double angleOffset = 0.3227539;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 9;
      public static final boolean invertDrive = true;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = -0.27124;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 12;
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = 0.15625;
      ;
      // -120.937
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(
              driveMotorID,
              angleMotorID,
              canCoderID,
              angleOffset,
              invertEncoder,
              invertDrive,
              invertSteer);
    }

    public static final int pigeonID = 13;
    public static final double trackWidth = 0.5647;
    public static final double wheelBase = 0.5647;
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

    public static final double DRIVE_BASE_RADIUS =
        Math.sqrt(wheelBase * wheelBase / 4 + trackWidth * trackWidth / 4);
    //public static final double WheelRadius = 0.0508;
    public static final double WheelRadius = 0.04826;

  }

  public static class ClimbConstants {
    public static Pose2d RightPoseBlue = new Pose2d(1.08+0.0254, 2.859+0.1, Rotation2d.fromDegrees(180));
    public static Pose2d RightPoseBlueAdjusted = new Pose2d(1.08 + 0.0254, 2.859, Rotation2d.fromDegrees(180));

    public static double supplyCurrentLimit = 60;
    public static double supplyCurrentLowerLimit = 40;
    public static double supplyCurrentLowerLimit_time = 0.25;
    public static double lowerLimit = 0;
    public static double upperLimit = 50;
  }

  public static class ShooterConstants {

    public static final double shooterL_kV = 0.1225;
    public static final double shooterL_kS = 0.375;
    public static final double shooterL_kP = 0.5;
    public static final double shooterL_kD = 0;

    public static final double shooterR_kV = 0.1225;
    public static final double shooterR_kS = 0.45;   
    public static final double shooterR_kP = 0.5;
    public static final double shooterR_kD = 0;
 
    public static final double supplyCurrentLimit = 60;
    public static final double supplyCurrentLowerLimit = 50;
    public static final double supplyCurrentLowerLimit_time = 0.2;
    public static final double statorCurrentLimit = 80;

    public static final double maxMovingSpeed = 0.1;
    
    //from looking from intake to shooter

    public static final boolean shooterL_inverted = false;
    public static final boolean shooterR_inverted = true;
    public static final boolean shooterPivot_inverted = true;
    public static final boolean feederConfig_inverted = false;

    public static final double shooterPivot_kP = 1.5;
    public static final double shooterPivot_kS = 0.35;
    public static final double shooterPivot_kD = 0.0;


    public static final double Pivot_HOME = 17.8;
    public static final double YawAngleTolerance = 2;
    public static final double ShooterVelocityTolerance = 75;

    //note that this value has to be MUCH higher bcs the shooter velocity drops by 200rpm+ when a ball is shot
    public static final double ShooterVelocity_NoGo_Tolerance = 1160000;

    //TODO: find corresponding encoder values for pivot that we can figure out what a good tolerance is

    public static final double ShooterPivotTolerance = 0.25;
    public static final double feederSupplyCurrentLimit = 50;



    //Shoot-on-the-fly constants
    public static final double z = 0 ;//405.559;
    public static final double y = 0;
    public static final double x = 0.244244;

    public static final Transform2d robotToShooter = new Transform2d(new Translation2d(x, y), new Rotation2d());
    public static final double HUB_SHOOTING_VELOCITY = 0;
    public static final double BASIC_PASSING_VELOCITY = 0;
    public static final double BASIC_PASSING_PIVOT = 0;
    public static final int pivot_gear_ratio = 25;
    public static final double abs_offset = 0.132714;
    public static final double MAX_ENCODER_VAL = 0.91;
    public static final double MIN_ENCODER_VAL = 0.15;
    public static final double ShootingManualDistance = 2.05;
    public static final double PassingManualDistance = 5.0;
    public static final double PassingVelocityTolerance = 0;
    public static final double PassingPivotTolerance = 0;

  }

  public static class IntakeConstants {
    public static final double pivot_kP_up = 2.5;
    public static final double pivot_kP_down = 1.15;
    public static final double pivot_kG = -0.155;
    public static final double offset = 0.42;
    public static final double PivotGearRatio = 45;
    public static final boolean pivot_inverted = true;
    public static final boolean intake_inverted = true;

    public static final double intakeSupplyCurrentLimit = 50;
    public static final double intakeStatorCurrentLimit = 40;
    public static final double HOME_Position = 0.43;
    public static final double INTAKING_Position = 0.097;
    public static final double SHUFFLE_UP_POSITION = 0.28;
   // public static final double BUMPING_Position = 25;
    public static final double perma_offset = 0.4311482857787071;
    public static final double cf_spring = 0.15;
    public static final double MAX_ENCODER_VAL = 0.44;
    public static final double MIN_ENCODER_VAL = 0.097;
    public static final double pivot_kI = 0;
  }

  public static class IndexerConstants {
    public static final double supplyCurrentLimit = 40;

  }


}
