package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Shooter.Shooter;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class shootingPathsAuto extends Command{
    Shooter shooter;
    Drive drive;
    Indexer indexer;
    PathPlannerPath path;
    public shootingPathsAuto(Shooter shooter, Drive drive, Indexer indexer,PathPlannerPath path) {
        this.drive =drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.path = path;
        addRequirements(shooter, drive, indexer);
    }
    public void initialize() {
        Translation2d currentPos = drive.getEstimatedPosition().getTranslation();
        Translation2d wantedPos = path.getStartingHolonomicPose().get().getTranslation();
        double distance = currentPos.getDistance(wantedPos);
        double speed = drive.getEstimatedPosition().getTranslation().getDistance(wantedPos) / 3;
        if(speed>Constants.ShooterConstants.maxMovingSpeed) {
            speed = Constants.ShooterConstants.maxMovingSpeed;
        }
        speed = 
    }

}
