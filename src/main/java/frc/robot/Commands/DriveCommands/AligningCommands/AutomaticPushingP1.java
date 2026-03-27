package frc.robot.Commands.DriveCommands.AligningCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class AutomaticPushingP1 extends Command {
    private final Drive drive;
  

    double angle_to_chase = 0;

    double kP_rotation;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final CommandXboxController controller;

    double trench_start_x = 4.57454;
    double half_x_field = 8.219694;
    double deltaRotationABS = 99999;

    boolean goingUp;
    boolean isOnBlueSide;


    public AutomaticPushingP1(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double kP_rotation, CommandXboxController controller) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.controller = controller;
        this.kP_rotation = kP_rotation;
        addRequirements(drive);
    }


    @Override
    public void initialize() {    

        if (drive.getEstimatedPosition().getX() < half_x_field) {
            isOnBlueSide = true;
        }

        else {
            isOnBlueSide = false;
        }

        if (drive.getFieldRelativeSpeeds().vyMetersPerSecond > 0) {
            goingUp = true;
        } 

        else {
            goingUp = false;
        }
       // intake.retractBump();

    }

    
    @Override
    public void execute() {

        if (isOnBlueSide && drive.getEstimatedPosition().getX() > (half_x_field + 2.0)) {
            isOnBlueSide = false;
        }

        if (!isOnBlueSide && drive.getEstimatedPosition().getX() < (half_x_field - 2.0)) {
            isOnBlueSide = true;
        }

        if (goingUp && drive.getFieldRelativeSpeeds().vyMetersPerSecond < -0.25) {
            goingUp = false;
        }

        if (!goingUp && drive.getFieldRelativeSpeeds().vyMetersPerSecond > 0.25) {
            goingUp = true;
        }

        double rotation;

       if (goingUp && isOnBlueSide) {
        rotation = Units.degreesToRadians(200);

       }

       else if (goingUp && !isOnBlueSide) {
        rotation = Units.degreesToRadians(20);
       }

       else if (!goingUp && isOnBlueSide) {
        rotation = Units.degreesToRadians(160);
       }

       else  {
        rotation = Units.degreesToRadians(-20);
       }

       if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
        rotation = rotation * -1;
       }



        double currentAngle = drive.getEstimatedPosition().getRotation().getRadians();
        double delta = MathUtil.angleModulus(rotation - currentAngle);
        double deltaDegrees = Math.toDegrees(delta);
        double omega = kP_rotation * deltaDegrees;
        deltaRotationABS = Math.abs(deltaDegrees);

        Translation2d linearVelocity;

        if (controller.rightStick().getAsBoolean()) {
          linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble() / 12, ySupplier.getAsDouble() / 12);
        }

        else {
            linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            
        

        }
        linearVelocity = linearVelocity.times(drive.getMaxLinearSpeedMetersPerSec());
        

               // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX(),
                      linearVelocity.getY(),
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








  

    

    
      
    
}
