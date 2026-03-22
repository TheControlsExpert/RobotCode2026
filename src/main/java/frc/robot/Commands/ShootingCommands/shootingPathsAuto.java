package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Shooter.Shooter;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class shootingPathsAuto extends Command{
    Shooter shooter;
    Drive drive;
    Indexer indexer;
    PathPlannerPath path;
    Drive swerve;
    public shootingPathsAuto(Shooter shooter, Drive drive, Indexer indexer,PathPlannerPath path) {
        this.drive =drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.path = path;
        addRequirements(shooter, drive, indexer);
    }
    public void initialize() {
        Translation2d velocityVector;
        Translation2d currentPos = drive.getEstimatedPosition().getTranslation();
        Translation2d wantedPos = path.getStartingHolonomicPose().get().getTranslation();
        Translation2d distance = wantedPos.minus(currentPos);
        double speed = currentPos.getDistance(wantedPos) / 3;
        if(speed>Constants.ShooterConstants.maxMovingSpeed) {
            speed = Constants.ShooterConstants.maxMovingSpeed;
        }
        if(distance.getNorm() > 0.01) {
            velocityVector = distance.times(speed/distance.getNorm());
        }
        else {
            velocityVector = new Translation2d();
        }

        if(DriverStation.getAlliance().get().equals(Alliance.Red)){
        velocityVector = velocityVector.unaryMinus();
        }
        ChassisSpeeds speeds = new ChassisSpeeds(velocityVector.getX()*swerve.getMaxLinearSpeedMetersPerSec(),velocityVector.getY()(swerve.getMaxLinearSpeedMetersPerSec()));
    }

}
