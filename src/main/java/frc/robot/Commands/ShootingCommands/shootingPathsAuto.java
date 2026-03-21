package frc.robot.Commands.ShootingCommands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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
    ChassisSpeeds MOVE;
    Timer timer = new Timer();
    double timeout = 1;


    public shootingPathsAuto(Shooter shooter, Drive drive, Indexer indexer, PathPlannerPath nextPath) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.nextPath = nextPath;
        addRequirements(shooter, drive, indexer);
    }
    

    public void initialize() {
        timer.restart();

        initialPos = drive.getEstimatedPosition().getTranslation();
        finalPos = nextPath.getStartingHolonomicPose().get().getTranslation();
        
        Translation2d distanceVector = finalPos.minus(initialPos);
        double rawSpeed = finalPos.getDistance(initialPos) / timeout;

        Translation2d linearVelocity = distanceVector.times(rawSpeed/distanceVector.getNorm());

        MOVE = new ChassisSpeeds(
                linearVelocity.getX(),
                linearVelocity.getY(), 0);
    }



    public boolean isFinished () {
        return timer.hasElapsed(timeout) && drive.getEstimatedPosition().getTranslation().getDistance(finalPos) < 0.01;
    }

    public void end (boolean interrupted) {
        //set the chassis speed to 0
    }

}
