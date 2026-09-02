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
import frc.robot.Robot.ElmoState;
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
    public boolean readyToShoot = false;
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
    double kD_rotation = 0;

    double prev_angleToTarget_radians = 0;
    double prev_timestamp = 0;

    boolean cant_shoot = true;

    

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
        prev_angleToTarget_radians = drive.getEstimatedPosition().getTranslation().minus(drive.calculateShootingPosition(0)).getAngle().getRadians();
        prev_timestamp = Timer.getFPGATimestamp();
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
    boolean isManual = Robot.elmoState.equals(ElmoState.ManualControl)
            || Robot.localizationState.equals(LocalizationState.DISABLED);

    // ── SHOOTER / HOOD ──────────────────────────────────────────────────
    if (isManual) {
        if (Robot.shootingState.equals(ShootingState.SHOOTING)) {
            shooter.shootManual(controller.getRightTriggerAxis()); //changed to right
        } else {
            shooter.passManual();
        }
    } else {
        if (Robot.shootingState.equals(ShootingState.SHOOTING)) {
            shooter.LookupTable_SOTM(drive, Timer.getFPGATimestamp() - prev_timestamp);
        } else {
            shooter.LookupTable_Passing(drive, timer.get());
        }
    }

    // ── DRIVE / ROTATION ────────────────────────────────────────────────
    Translation2d linearVelocity;

    if (controller.rightStick().getAsBoolean()) {
        linearVelocity = getLinearVelocityFromJoysticks(
                xSupplier.getAsDouble() / 12, ySupplier.getAsDouble() / 12);
    } else {
        linearVelocity = getLinearVelocityFromJoysticks(
                xSupplier.getAsDouble(), ySupplier.getAsDouble());
    }

    double omega;
    double angleToTarget_radians;
    double distance;

    if (isManual) {
        // Free rotation from joystick
        omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.2);
        if (controller.rightStick().getAsBoolean()) omega /= 12;
        omega = Math.copySign(omega * omega, omega);
        omega *= drive.getMaxAngularSpeedRadPerSec();

        distance = ShooterConstants.ShootingManualDistance;
        angleToTarget_radians = 0; // unused
    } else {
        // Auto-aim
        Translation2d shootingPosition = drive.calculateShootingPosition(timer.get());
        distance = drive.getEstimatedPosition().getTranslation().getDistance(shootingPosition);

        double deltaTime = Timer.getFPGATimestamp() - prev_timestamp;

        if (Robot.shootingState.equals(ShootingState.SHOOTING)) {
            angleToTarget_radians = shooter.LookupTable_SOTM(drive, deltaTime).getRadians();
        } else {
            angleToTarget_radians = drive.calculateShootingPosition(timer.get())
                    .minus(drive.getEstimatedPosition().getTranslation())
                    .getAngle().getRadians();
        }

        double derivativeAddon = (angleToTarget_radians - prev_angleToTarget_radians)
                / Math.max(deltaTime, 1e-6);
        prev_angleToTarget_radians = angleToTarget_radians;
        prev_timestamp = Timer.getFPGATimestamp();

        double deltaRotation = MathUtil.angleModulus(
                angleToTarget_radians - drive.getEstimatedPosition().getRotation().getRadians());
        deltaRotation = Math.toDegrees(deltaRotation);
        SmartDashboard.putNumber("delta angle yaw", deltaRotation);

        omega = MathUtil.clamp(
                deltaRotation * kP_rotation + derivativeAddon * kD_rotation,
                -drive.getMaxAngularSpeedRadPerSec(),
                drive.getMaxAngularSpeedRadPerSec());
    }

    boolean isFlipped = DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega),
            isFlipped
                    ? drive.getEstimatedPosition().getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getEstimatedPosition().getRotation()));

    // ── READY TO SHOOT CHECK ─────────────────────────────────────────────
    if (isManual) {

        readyToShoot = true;

    } else {
        double deltaRotation = isManual ? 0 :
                Math.toDegrees(MathUtil.angleModulus(
                        angleToTarget_radians - drive.getEstimatedPosition().getRotation().getRadians()));

        boolean aimReady = Math.abs(deltaRotation) < 7.5;
        boolean shooterReady = shooter.isAtShootingVelocity(distance)
                && shooter.isAtPivotPosition(distance);
        boolean passingMode = Robot.shootingState.equals(ShootingState.PASSING);

        if (!readyToShoot && drive.getGyroSpeed() < 10
                && (shooterReady || passingMode) && aimReady) {
            readyToShoot = true;
            shooter.isShooting = true;
            RobotContainer.isShooting = true;
            SmartDashboard.putBoolean("Shooter is at Velocity", true);
        }
    }

    // ── FEED ─────────────────────────────────────────────────────────────
    if (readyToShoot) {
        indexer.setIndexerDutyCycle(1);
        shooter.setFeederVelocity(1);
    } else {
        indexer.setIndexerDutyCycle(0);
        shooter.setFeederVelocity(0);
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
    //shooter.setShooterVelocity(0);
   // shooter.setPositionPivot(ShooterConstants.Pivot_HOME);
   // shooter.isShooting = false;
    //RobotContainer.isShooting = false;
 //   }

    CommandScheduler.getInstance().schedule(new Jam(indexer, shooter, 0.65)); //runs the indexer in the opposite direction to clear balls from the shooter
   

    
    //CommandScheduler.getInstance().cancel(shuffle);
}


@Override
public boolean isFinished() {
    return timer.hasElapsed(timeout);
}
}


  
