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
import edu.wpi.first.math.geometry.Translation2d;
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

  public static class SwerveConstants {

    // Gear Ratio
    public static final double driveReduction = 5.35;
    public static final double steerReduction = 18.75;

    /* Steer Motor PID Values */

    public static final double angleKP = 25;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKS = 0.45;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.01; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.156; // TODO: This must be tuned to specific robot
    public static final double driveKV = 0.60;
    public static final double driveKA = 0.1;

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
      public static final boolean invertDrive = true;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = -0.054;
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
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;

      public static final double angleOffset = -0.2019;
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
      public static final boolean invertDrive = false;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = 0.406;
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
      public static final boolean invertDrive = true;
      public static final boolean invertSteer = true;
      public static final boolean invertEncoder = false;
      public static final double angleOffset = -0.02832;
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

    public static final double trackWidth = 0.632177;
    public static final double wheelBase = 0.62081;
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

    public static final double DRIVE_BASE_RADIUS =
        Math.sqrt(wheelBase * wheelBase / 4 + trackWidth * trackWidth / 4);
    public static final double WheelRadius = 0.0508;
    //public static final double WheelRadius = 0.0477;
    public static final LinearVelocity MaxFreeSpeed = null;
    public static final double odometryConstant = 0;
    public static final double maxAccel = 0;
    public static final double maxJerk = 1.2;
    public static final double collisionMultiplier = 0.1;
    public static final double kAccel = 0.05;
    public static final double kMovement = 0.45;

  }


  public static class FieldConstants {
    //BLUE

   
    
    public static final Pose2d RightSource_BLUE = new Pose2d(1.34, 7.18, Rotation2d.fromDegrees(130));
    //public static final Pose2d LeftSource_BLUE = new Pose2d(1.277, 1.053, Rotation2d.fromDegrees(-130));

    //public static final Pose2d RightSource_RED = new Pose2d(16.54, 7.076, Rotation2d.fromDegrees(50));
    public static final Pose2d LeftSource_RED  = new Pose2d(16.45, 1.33 , Rotation2d.fromDegrees(-50));

    public static final double REEF_Y_OFFSET = 0.15;
    public static final double REEF_X_OFFSET = 0.33;

    public static final double REEF_Y_OFFSET_ALGAE = 0;
    public static final double REEF_X_OFFSET_ALGAE = 0.33;

    public static final double REEF_Y_OFFSET_STEPBACK = 0;
    public static final double REEF_X_OFFSET_STEPBACK = 1;





  }


  public static class ElevatorConstants {
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double TargetAcceleration = 0;
    public static final double TargetVelocity = 0;
    //left when looking from back side (intake side)
    public static final int IDLeft = 14;
    public static final int IDRight = 13;
    public static final double toleranceElevator = 1;

    public static final double maxHeightAccel = 2;
    public static final double maxHeight = 25;

    public static final double maxHeightAccel2 = 1;
    public static final double maxHeight2 = 43;





    public static final double minHeightAccel = 4.5;
    public static final double minHeight = 0.5;
    public static final double minHeightforL4Pivot = 0;
    public static final double minHeightAboveHome = 0;


  }

  

  public static class WristConstants {
    public static final int ID_Intake = 0;
    public static final int ID_Pivot = 1;
    public static final int ID_Wrist = 2;
    public static final double rotorToSensorRatio = 0;
    public static final double offsetPivot = 0;
    public static final double AccelerationMotionMagic = 0;
    public static final double CruisingVelocityMotionMagic = 0;

    public static final double kP = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kP_wrist = 0;
    public static final double tolerancePivot = 1;
    public static final double toleranceWrist = 1;


  }


  public static class IntakeConstants {

    public static final double currentMax = 0;

  }
}
