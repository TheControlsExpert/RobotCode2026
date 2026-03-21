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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Commands.IntakeCommands.Jam;
import frc.robot.Commands.IntakeCommands.ShuffleCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot.LocalizationState;
import frc.robot.Robot.ShootingState;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class Shooting extends Command {
    Drive drive;
    Shooter shooter;
    Indexer indexer;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    DoubleSupplier rotationSupplier;
    CommandXboxController controller;
    double kP_rotation;
    boolean readyToShoot = false;
    Command shuffle;
    IntakeSubsystem intake;
    boolean hasShuffled = false;
    boolean waiting = false;
    Timer timer = new Timer();
    Timer shuffleTimer = new Timer();
    double timeout = 9999;


    double maxFuelCountDelay = 1.5;
    double minFuelCountDelay = 0;
    double shiftEndFuelCountExtension = 3.0;
    double bps = 7;
    VisionSubsystem vision;
    boolean isShuffling = true;

    

    public Shooting(Shooter shooter, Drive drive, Indexer indexer, IntakeSubsystem intake, CommandXboxController controller, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, double kP_rotation, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.drive = drive;
        this.indexer = indexer;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;
        this.kP_rotation = kP_rotation;
        this.intake = intake;
        addRequirements(shooter, drive, indexer);
        
    }

    public Shooting(Shooter shooter, Indexer indexer, Drive drive) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
        addRequirements(shooter, indexer, drive);
    }

     public Shooting(Shooter shooter, Drive drive, Indexer indexer, IntakeSubsystem intake, CommandXboxController controller, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, double kP_rotation, double timeout, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.drive = drive;
        this.indexer = indexer;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;
        this.kP_rotation = kP_rotation;
        this.intake = intake;
        this.timeout = timeout;
        addRequirements(shooter, drive, indexer);
        
    }


    @Override
    public void initialize() {
        vision.ShootingMode(true);   
        readyToShoot = false;
        hasShuffled = false;
        isShuffling = true;
        waiting = false;
        timer.restart();
        shuffleTimer.restart();
    }


    @Override
    public void execute() {
      
     
       // SmartDashboard.putBoolean("Shooting shuffle", isShuffling);

        // if (shuffleTimer.hasElapsed(2) && isShuffling) {
        //     isShuffling = false;
        //     shuffleTimer.restart();
        //    // indexer.setIndexerDutyCycle(1);
        // }

        // if (shuffleTimer.hasElapsed(1) && !isShuffling) {
        //     isShuffling = true;
        //     shuffleTimer.restart();
        // }


        // if (isShuffling) {
        //     indexer.setIndexerDutyCycle(-0.3);
        //     shooter.setFeederVelocity(-0.3);
        // }

        // else {
          //  indexer.setIndexerDutyCycle(1);
           // shooter.setFeederVelocity(1);
    //   }




















        if (Robot.localizationState.equals(LocalizationState.DISABLED)) {
          Translation2d linearVelocity;

        if (controller.rightStick().getAsBoolean()) {
          linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble() / 12, ySupplier.getAsDouble() / 12);
        }

        else {
            linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        }

              // Calculate angular speed
              double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.2);

         if (controller.rightStick().getAsBoolean()) {
             omega = omega / 12;
         }

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);
           boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
             
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
        
    if (Robot.shootingState.equals(ShootingState.SHOOTING)) {
    shooter.shootManual();
    }

    else {
    shooter.passManual();
    }
    

    if ((Robot.isActive && (Robot.combinedTimeLeft + shiftEndFuelCountExtension - maxFuelCountDelay - shooter.getMaxTOF() - 1/bps) > 0) || 
    (!Robot.isActive && (shooter.getMinTOF() +  minFuelCountDelay - Robot.combinedTimeLeft) > 0) || 
    (Robot.shootingState.equals(ShootingState.PASSING)) ||
    (!Robot.winner_selection_done)) {
    if (shooter.isAtShootingVelocity(ShooterConstants.ShootingManualDistance) && shooter.isAtPivotPosition(ShooterConstants.ShootingManualDistance)) {
        readyToShoot = true;
        RobotContainer.isShooting = true;
        shooter.isShooting = true;
        SmartDashboard.putBoolean("Shooter is at Velocity", true);
    }

    }


else {
    readyToShoot = false;
}

 if (readyToShoot) {
        indexer.setIndexerDutyCycle(1);
        shooter.setFeederVelocity(1);
    }
 else {
        indexer.setIndexerDutyCycle(0);
        shooter.setFeederVelocity(0);
 }   


 }

    else {
        if (Robot.shootingState.equals(ShootingState.SHOOTING)) {
        shooter.LookupTable_Shooting(drive);
        }

        else {
        shooter.LookupTable_Passing(drive);    
        }

        Translation2d linearVelocity;

        if (DriverStation.isTeleop()) {

        if (controller.rightStick().getAsBoolean()) {
          linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble() / 12, ySupplier.getAsDouble() / 12);
        }

        else {
            linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        }
    }

    else {
        linearVelocity = new Translation2d(0, 0);
    }

              
        Translation2d shootingPosition = drive.calculateShootingPosition();

        double distance = drive.getEstimatedPosition().getTranslation().getDistance(shootingPosition);
        //double[] shootingParameters = shooter.LookupTable_Shooting(drive);
        double omega;
        double angleToTarget_radians = shootingPosition.minus(drive.getEstimatedPosition().getTranslation()).getAngle().getRadians();
        double deltaRotation = angleToTarget_radians - drive.getEstimatedPosition().getRotation().getRadians();
        
        deltaRotation = MathUtil.angleModulus(deltaRotation);
        //Change back to degrees
        deltaRotation = Math.toDegrees(deltaRotation);

     //   if (Robot.localizationState.equals(LocalizationState.OPERATIONAL)) {
           // shooter.setShooterVelocity(shootingParameters[0]);
           // shooter.setPositionPivot(shootingParameters[1]);

        
         omega = deltaRotation * kP_rotation;
        
    //    }

//         else {
//             omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.2);

//             if (controller.rightStick().getAsBoolean()) {
//                 omega = omega / 12;
//             }

//           // Square rotation value for more precise control
//             omega = Math.copySign(omega * omega, omega);
 
//             if (Robot.shootingState.equals(ShootingState.SHOOTING)) {
//                 shooter.setShooterVelocity(ShooterConstants.HUB_SHOOTING_VELOCITY);
//                 shooter.setPositionPivot(ShooterConstants.Pivot_HOME);
//             }
//              else if (Robot.shootingState.equals(ShootingState.PASSING)) {
//                 shooter.setShooterVelocity(ShooterConstants.BASIC_PASSING_VELOCITY);
//                 shooter.setPositionPivot(ShooterConstants.BASIC_PASSING_PIVOT);
//             }
//         }
        

//               // Convert to field relative speeds & send command
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
    
if ((Robot.isActive && (Robot.combinedTimeLeft + shiftEndFuelCountExtension - maxFuelCountDelay - shooter.getMaxTOF() - 1/bps) > 0) || 
    (!Robot.isActive && (shooter.getMinTOF() +  minFuelCountDelay - Robot.combinedTimeLeft) > 0) || 
    (Robot.shootingState.equals(ShootingState.PASSING)) ||
    (!Robot.winner_selection_done)) {

    //shooting parameters are close enough to START shooting
    if (!readyToShoot && ((shooter.isAtShootingVelocity(distance) && shooter.isAtPivotPosition(distance)) || Robot.shootingState.equals(ShootingState.PASSING)) && drive.getGyroSpeed() < 2 && (Math.abs(deltaRotation) < ShooterConstants.YawAngleTolerance)) {
        readyToShoot = true;
        shooter.isShooting = true;
        RobotContainer.isShooting = true;
        SmartDashboard.putBoolean("Shooter is at Velocity", true);
        
    }
//shooting parameters are too far, STOP shooting
    // if (shooter.isShooterVelocityLow(distance) && readyToShoot && DriverStation.isTeleop() && Robot.shootingState.equals(ShootingState.SHOOTING)) {
    //     readyToShoot = false;
    //     waiting = true;
    // }

  
        SmartDashboard.putBoolean("Shooter is at Velocity", readyToShoot);
 //   }
}

else {
    readyToShoot = false;
}


    if (readyToShoot) {
        indexer.setIndexerDutyCycle(1);
        shooter.setFeederVelocity(1);


    }

    else {
            indexer.setIndexerDutyCycle(0);
            shooter.setFeederVelocity(0);
    }
        // if (shuffleTimer.hasElapsed(1) && !isShuffling) {
        //     isShuffling = true;
        //     shuffleTimer.restart();
        //     indexer.setIndexerDutyCycle(-1);
        // }

        // if (shuffleTimer.hasElapsed(0.2) && isShuffling && hasShuffled || shuffleTimer.hasElapsed(0.2) && isShuffling && !hasShuffled) {
        //     isShuffling = false;
        //     hasShuffled = true;
        //     shuffleTimer.restart();
          
        // }


        // if (!isShuffling) {
          //  indexer.setIndexerDutyCycle(1);
          //  shooter.setFeederVelocity(0.75);

        }
    }

        // else {
        //     indexer.setIndexerDutyCycle(-1);
        //     shooter.setFeederVelocity(0.75);
        // }
      
   // }
//
   // else if (waiting){
   //     indexer.setIndexerDutyCycle(-0.5);
    //    shooter.setFeederVelocity(-0.5);
   // }

    // if (!intake.isHopperFull() && intake.isReadyToClose() && !hasShuffled && DriverStation.isAutonomous() && !intake.is_busy) {
    //     CommandScheduler.getInstance().schedule(shuffle);
    //     hasShuffled = true;
    // }

    //if (timer.hasElapsed(2)) {
       // intake.beep = true;
   // }


// else {
//     indexer.setIndexerDutyCycle(0);
//     shooter.setFeederVelocity(0);
//     readyToShoot = false;
//}
//}  


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
    // if (!DriverStation.isAutonomous()) {
    vision.ShootingMode(false);
    shooter.setShooterVelocity(0);
    shooter.setPositionPivot(ShooterConstants.Pivot_HOME);
 //   }

    //CommandScheduler.getInstance().schedule(new Jam(indexer, shooter, 2.0)); //runs the indexer in the opposite direction to clear balls from the shooter
   
    indexer.setIndexerDutyCycle(0);
    shooter.setFeederVelocity(0);
    //CommandScheduler.getInstance().cancel(shuffle);
}


@Override
public boolean isFinished() {
    return timer.hasElapsed(timeout);
}
}


  
