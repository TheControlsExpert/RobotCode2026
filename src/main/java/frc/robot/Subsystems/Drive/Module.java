package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.Subsystems.Drive.ModuleIO.ModuleIOInputs;

import org.littletonrobotics.junction.Logger;

public class Module {
  int sampleCount = 0;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveConstants.SwerveModuleConstants constants;

  private boolean wasDisconnectedDrive = false;
  private boolean wasDisconnectedTurn = false;
  private boolean wasDisconnectedTurnEncoder = false;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index, SwerveConstants.SwerveModuleConstants constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
  
  }

  public void periodic() {
    io.updateInputs(inputs);
    //Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    if (RobotBase.isReal()) {
    sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * SwerveConstants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
   }

   else {
      double positionMeters = inputs.drivePositionRad * SwerveConstants.WheelRadius;
      Rotation2d angle = inputs.turnAbsolutePosition;
      odometryPositions = new SwerveModulePosition[] {new SwerveModulePosition(positionMeters, angle)};
   }

    // Update alerts

    if (DriverStation.isDisabled()) {
      if (!wasDisconnectedDrive && !inputs.driveConnected) {
        Robot.reportDisconnection("Drive Motor " + index);
        wasDisconnectedDrive = true;
      }

      if (wasDisconnectedDrive && inputs.driveConnected) {
        Robot.removeDisconnection("Drive Motor " + index);
        wasDisconnectedDrive = false;
      }
      if (!wasDisconnectedTurn && !inputs.turnConnected) {
        Robot.reportDisconnection("Turn Motor " + index);
        wasDisconnectedTurn = true;
      }
      if (wasDisconnectedTurn && inputs.turnConnected) {
        Robot.removeDisconnection("Turn Motor " + index);
        wasDisconnectedTurn = false;
      }
      if (!wasDisconnectedTurnEncoder && !inputs.turnEncoderConnected) {
        Robot.reportDisconnection("CANCoder " + index);
        wasDisconnectedTurnEncoder = true;
      }
      if (wasDisconnectedTurnEncoder && inputs.turnEncoderConnected) {
        Robot.removeDisconnection("CANCoder " + index);
        wasDisconnectedTurnEncoder = false;
      }
  }
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    //state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond /SwerveConstants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  public double getPositionRadians() {
    return inputs.drivePositionRad;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

// 
  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * SwerveConstants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * SwerveConstants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  public double getFFCharacterizationAcceleration() {
    return Units.radiansToRotations(inputs.driveAccelerationRadPerSec);
  }
}