package frc.robot.Commands.ShootingCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Shooter.Shooter;

public class Shooting extends Command {
    Drive drive;
    Shooter shooter;
    Indexer indexer;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    CommandXboxController controller;
    double kP_rotation;
    boolean readyToShoot = false;


    public Shooting(Shooter shooter, Drive drive, Indexer indexer, CommandXboxController controller, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double kP_rotation) {
        this.shooter = shooter;
        this.drive = drive;
        this.indexer = indexer;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.controller = controller;
        this.kP_rotation = kP_rotation;
        addRequirements(shooter, drive, indexer);
        
    }

    @Override
    public void initialize() {
        readyToShoot = false;
    }

    @Override
    public void execute() {
                Translation2d linearVelocity;

        if (controller.rightStick().getAsBoolean()) {
          linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble() / 12, ySupplier.getAsDouble() / 12);
        }

        else {
            linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        }

              
        Translation2d shootingPosition = drive.calculateShootingPosition();

        double distance = drive.getEstimatedPosition().getTranslation().getDistance(shootingPosition);
        shooter.LookupTable_Shooting(distance);

        double angleToTarget_radians = shootingPosition.minus(drive.getEstimatedPosition().getTranslation()).getAngle().getRadians();
        double deltaRotation = angleToTarget_radians - drive.getEstimatedPosition().getRotation().getRadians();
        
        deltaRotation = MathUtil.angleModulus(deltaRotation);
        //Change back to degrees
        deltaRotation = Math.toDegrees(deltaRotation);
        double omega = deltaRotation * kP_rotation;


              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                   MathUtil.clamp(omega, -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec()));
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getEstimatedPosition().getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getEstimatedPosition().getRotation()));
    


    //
    if (!readyToShoot && shooter.isAtShootingVelocity(distance) && shooter.isAtPivotPosition(distance) && Math.abs(deltaRotation) < ShooterConstants.YawAngleTolerance) {
        readyToShoot = true;
        
    }


    if (readyToShoot) {
        indexer.setIndexerDutyCycle(1);
        shooter.setFeederVelocity(0.6);
    }
}  


    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }


  




    
@Override
public void end(boolean interrupted) {
    shooter.setShooterVelocity(0);
    shooter.setPositionPivot(ShooterConstants.Pivot_HOME);
    indexer.setIndexerDutyCycle(0);
}
}


  
