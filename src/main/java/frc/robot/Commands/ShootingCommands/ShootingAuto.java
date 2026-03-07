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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.IntakeCommands.ShuffleCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShootingAuto extends Command {
    Drive drive;
    Shooter shooter;
    Indexer indexer;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    CommandXboxController controller;
    double kP_rotation;
    boolean readyToShoot = false;
    ShuffleCommand shuffle;
    IntakeSubsystem intake;
    boolean hasShuffled = false;
    boolean waiting = false;
    Translation2d targetPosition;
    double speed;
    Timer timer = new Timer();
    double timeout;
    public ShootingAuto(Shooter shooter, Drive drive, Indexer indexer, IntakeSubsystem intake, double kP_rotation, ShuffleCommand shuffle, double timeout, Translation2d targetPosition) {
        this.shooter = shooter;
        this.drive = drive;
        this.indexer = indexer;
        this.kP_rotation = kP_rotation;
        this.shuffle = shuffle;
        this.timeout = timeout;
        this.intake = intake;
        this.targetPosition = targetPosition;
        addRequirements(shooter, drive, indexer);
        
    }

    @Override
    public void initialize() {
        readyToShoot = false;
        hasShuffled = false;
        waiting = false;
        timer.restart();
    }


    @Override
    public void execute() {

       

              
        Translation2d shootingPosition = drive.calculateShootingPosition();

        double distance = drive.getEstimatedPosition().getTranslation().getDistance(shootingPosition);
        shooter.LookupTable_Shooting(drive);

        double angleToTarget_radians = shootingPosition.minus(drive.getEstimatedPosition().getTranslation()).getAngle().getRadians();
        double deltaRotation = angleToTarget_radians - drive.getEstimatedPosition().getRotation().getRadians();
        
        deltaRotation = MathUtil.angleModulus(deltaRotation);
        //Change back to degrees
        deltaRotation = Math.toDegrees(deltaRotation);
        double omega = deltaRotation * kP_rotation;

        Translation2d distance2 = targetPosition.minus(drive.getEstimatedPosition().getTranslation());
        Translation2d linearVelocity;

        speed = drive.getEstimatedPosition().getTranslation().getDistance(targetPosition) / timeout; //sets the speed the bot will be moving at
        if (speed > ShooterConstants.maxMovingSpeed) { speed = ShooterConstants.maxMovingSpeed; }

        if (distance2.getNorm() > 0.01) { // Prevent division by zero
     linearVelocity = distance2.times(speed/distance2.getNorm());}

        else {
     linearVelocity = new Translation2d();
        }

        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            linearVelocity = linearVelocity.unaryMinus();
        }
        




              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() ,
                      linearVelocity.getY() ,
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
        SmartDashboard.putBoolean("Shooter is at Velocity", true);
        
    }

    if (shooter.isShooterVelocityLow(distance) && readyToShoot) {
        readyToShoot = false;
        waiting = true;
    }

    if (!readyToShoot) {
        SmartDashboard.putBoolean("Shooter is at Velocity", false);
    }


    if (readyToShoot) {
        indexer.setIndexerDutyCycle(1);
        shooter.setFeederVelocity(1);
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
    if (!DriverStation.isAutonomous()) {
    shooter.setShooterVelocity(0);
    }
    shooter.setPositionPivot(ShooterConstants.Pivot_HOME);
    indexer.setIndexerDutyCycle(0);
}

@Override
public boolean isFinished() {
    return timer.hasElapsed(timeout);
}

}