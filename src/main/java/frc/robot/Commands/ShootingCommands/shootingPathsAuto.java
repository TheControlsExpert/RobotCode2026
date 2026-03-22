package frc.robot.Commands.ShootingCommands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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

    Pose2d initialPose;
    Pose2d finalPose;
    PathPlannerPath nextPath;
    ChassisSpeeds MOVE;
    Timer timer = new Timer();
    double timeout = 1;
    double kP_Rotation;


    public shootingPathsAuto(Shooter shooter, Drive drive, Indexer indexer, PathPlannerPath nextPath, double kP_Rotation) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.nextPath = nextPath;
        this.kP_Rotation = kP_Rotation;
        addRequirements(shooter, drive, indexer);
    }
    

    public void initialize() {
        timer.restart();

        initialPose = drive.getEstimatedPosition();
        finalPose = nextPath.getStartingHolonomicPose().get();
        
        //all the velocity stuff
        Translation2d distanceVector = finalPose.getTranslation().minus(initialPose.getTranslation()); //distance vector
        double rawSpeed = finalPose.getTranslation().getDistance(initialPose.getTranslation()) / timeout; //scalar velocity
        Translation2d linearVelocity = distanceVector.times(rawSpeed/distanceVector.getNorm()); //now the velocity is vector

        double targetRotation = finalPose.getTranslation().minus(initialPose.getTranslation()).getAngle().getRadians();
        double deltaRotation = targetRotation - drive.getEstimatedPosition().getRotation().getRadians();
        deltaRotation = MathUtil.angleModulus(deltaRotation);
        double Ilan = deltaRotation * kP_Rotation;

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
