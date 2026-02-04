package frc.robot.Commands.DriveCommands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive.Drive;

public class DriveCommand extends Command {

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private Drive swervyyy;
    private DoubleSupplier rotationSupplier;
    CommandXboxController controller;
        
    public DriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, Drive swervyyy, CommandXboxController controller) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.swervyyy = swervyyy;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;
        addRequirements(swervyyy);
    }

    @Override
    public void execute() {
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
              double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.1);

         if (controller.rightStick().getAsBoolean()) {
             omega = omega / 12;
         }

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * swervyyy.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * swervyyy.getMaxLinearSpeedMetersPerSec(),
                      omega * swervyyy.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              swervyyy.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? swervyyy.getRotation().plus(new Rotation2d(Math.PI))
                          : swervyyy.getRotation()));
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
