package frc.robot.Commands.DriveCommands.AligningCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;

public class AutoBumping extends Command {
    private final Drive drive;
    double angle1 = Math.toRadians(45);
    double angle2 = Math.toRadians(135);
    double angle3 = Math.toRadians(225);
    double angle4 = Math.toRadians(315);

    double angle_to_chase = 0;

    double kP_rotation;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final CommandXboxController controller;

    public AutoBumping(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double kP_rotation, CommandXboxController controller) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.controller = controller;
        this.kP_rotation = kP_rotation;
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        double currentAngle = drive.getEstimatedPosition().getRotation().getRadians();
        
        double smallestAngle = -1;
        double deltaRot = 0;

        for (double angle : new double[]{angle1, angle2, angle3, angle4}) {
            double delta = MathUtil.angleModulus(angle - currentAngle);
            if (smallestAngle == -1 || Math.abs(delta) < Math.abs(deltaRot)) {
                smallestAngle = angle;
                deltaRot = delta;
            }

        }

        angle_to_chase = smallestAngle;
        
    }

    
    @Override
    public void execute() {

        double currentAngle = drive.getEstimatedPosition().getRotation().getRadians();
        double delta = MathUtil.angleModulus(angle_to_chase - currentAngle);
        double deltaDegrees = Math.toDegrees(delta);
        double omega = kP_rotation * deltaDegrees;

        Translation2d linearVelocity;

        if (controller.rightStick().getAsBoolean()) {
          linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble() / 12, ySupplier.getAsDouble() / 12);
        }

        else {
            linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        }

               // Convert to field relative speeds & send command
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
        drive.runVelocity(new ChassisSpeeds());
     }
      
    
}
