package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot.ShootingState;
import frc.robot.RobotContainer;
import frc.robot.Commands.IntakeCommands.Jam;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Vision.VisionSubsystem;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class shootingPathsAuto extends Command{
    Shooter shooter;
    Indexer indexer;
    PathPlannerPath path;
    Drive swerve;
    VisionSubsystem vision;
    Timer timer = new Timer();
    double TimeToFinish = 4.25;
    Translation2d velocityVector;

    public shootingPathsAuto(Shooter shooter, Drive drive, Indexer indexer,PathPlannerPath path,VisionSubsystem vision) {
        this.swerve = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.path = path;
        this.vision = vision;
        addRequirements(shooter, drive, indexer);
    }

    public void initialize() {
        vision.ShootingMode(true);
        timer.restart();

        Translation2d currentPos = swerve.getEstimatedPosition().getTranslation();
        Translation2d wantedPos = path.getStartingHolonomicPose().get().getTranslation();
       
        Translation2d distance = wantedPos.minus(currentPos);
        double speed = currentPos.getDistance(wantedPos) / TimeToFinish;
        if(speed > 3) {
            speed = 3;
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
    }

    public void execute() {
        

        Rotation2d currentAngle = swerve.getEstimatedPosition().getRotation(); //just getting the omega (rotation)
        Rotation2d wantedAngle = shooter.LookupTable_SOTM(swerve,0.0);    
        Rotation2d currentToWanted = wantedAngle.minus(currentAngle);
        double RadianDistance = currentToWanted.getRadians();
        RadianDistance = MathUtil.angleModulus(RadianDistance);
        RadianDistance = Math.toDegrees(RadianDistance);
        double omega = RadianDistance*swerve.rotationkP;


        ChassisSpeeds speeds = new ChassisSpeeds
        (velocityVector.getX(),
        velocityVector.getY(),
        omega);
        boolean isFlipped =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        swerve.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                          : swerve.getRotation()));
        boolean readyToShoot = false;

        Translation2d shootingPosition = swerve.calculateShootingPosition(timer.get());

        double distance_to_hub = swerve.getEstimatedPosition().getTranslation().getDistance(shootingPosition);

        if (!readyToShoot &&  Math.abs(currentToWanted.getRadians()) < 10) {
            readyToShoot = true;
            shooter.isShooting = true;
            RobotContainer.isShooting = true;
            SmartDashboard.putBoolean("Shooter is at Velocity", true);
        }

        else {
            readyToShoot = false;
            shooter.isShooting = false;
            RobotContainer.isShooting = false;
            SmartDashboard.putBoolean("Shooter is at velocity", false);
        }
        

        if(readyToShoot) {
            shooter.setFeederVelocity(1);
            indexer.setIndexerDutyCycle(1);
        }
        else{
            shooter.setFeederVelocity(0);
            indexer.setIndexerDutyCycle(0);
        }

    }

    public boolean isFinished() {
        return timer.hasElapsed(TimeToFinish+0.5);

    }
    public void end(boolean interupted){
        swerve.runVelocity(new ChassisSpeeds());
        vision.ShootingMode(  false);
       // shooter.setShooterVelocity(0);
       // shooter.setPositionPivot(ShooterConstants.Pivot_HOME);


    }
}
