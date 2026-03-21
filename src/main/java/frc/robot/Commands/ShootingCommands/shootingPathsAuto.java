package frc.robot.Commands.ShootingCommands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Shooter.Shooter;

public class shootingPathsAuto extends Command{

    Shooter shooter;
    Drive drive;
    Indexer indexer;

    Translation2d initialPos;
    Translation2d finalPos;

    PathPlannerPath nextPath;

    public shootingPathsAuto(Shooter shooter, Drive drive, Indexer indexer, PathPlannerPath nextPath) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.nextPath = nextPath;
        addRequirements(shooter, drive, indexer);
    }
    public void initialize() {
        initialPos = drive.getEstimatedPosition().getTranslation();
        finalPos = nextPath.getStartingHolonomicPose().get().getTranslation();
        
        Translation2d distanceVector = finalPos.minus(initialPos);
        double rawSpeed = finalPos.getDistance(initialPos) / 4;

        Translation2d linearVelocity = distanceVector.times(rawSpeed/distanceVector.getNorm());


    }

}
