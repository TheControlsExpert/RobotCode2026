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
import frc.robot.Subsystems.Intake.IntakeSubsystem;

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

    double trench_start_x = 4.57454;
    double half_x_field = 8.219694;
    double deltaRotationABS = 99999;

    double maxAllowedSpeed = 3;
    private final IntakeSubsystem intake;

    public AutoBumping(Drive drive, IntakeSubsystem intake, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double kP_rotation, CommandXboxController controller) {
        this.drive = drive;
        this.intake = intake;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.controller = controller;
        this.kP_rotation = kP_rotation;
        addRequirements(drive, intake);
    }


    @Override
    public void initialize() {
       // intake.retractBump();

        deltaRotationABS = 99999;
        double currentAngle = drive.getEstimatedPosition().getRotation().getRadians() + (DriverStation.getAlliance().get().equals(Alliance.Red) ? Math.PI : 0);

        double smallestAngle = -1;
        double deltaRot = 0;

        if ((DriverStation.getAlliance().get().equals(Alliance.Blue) && drive.getEstimatedPosition().getX() > trench_start_x) ||
            (DriverStation.getAlliance().get().equals(Alliance.Red) && drive.getEstimatedPosition().getX() <  2 * half_x_field - trench_start_x)) {

        for (double angle : new double[]{angle2, angle3}) {
            double delta = MathUtil.angleModulus(angle - currentAngle);
            if (smallestAngle == -1 || Math.abs(delta) < Math.abs(deltaRot)) {
                smallestAngle = angle;
                deltaRot = delta;
            }

        }


        angle_to_chase = smallestAngle + (DriverStation.getAlliance().get().equals(Alliance.Red) ? Math.PI : 0);
        }

        else {
            for (double angle : new double[]{angle1, angle4}) {
                double delta = MathUtil.angleModulus(angle - currentAngle);
                if (smallestAngle == -1 || Math.abs(delta) < Math.abs(deltaRot)) {
                    smallestAngle = angle;
                    deltaRot = delta;
                }
    
            }
    
    
            angle_to_chase = smallestAngle + (DriverStation.getAlliance().get().equals(Alliance.Red) ? Math.PI : 0);
        }
        
    }

    
    @Override
    public void execute() {

        double currentAngle = drive.getEstimatedPosition().getRotation().getRadians();
        double delta = MathUtil.angleModulus(angle_to_chase - currentAngle);
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
        linearVelocity = linearVelocity.times(linearVelocity.getNorm() > maxAllowedSpeed ? maxAllowedSpeed / linearVelocity.getNorm() : 1);

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



  @Override
  public void end(boolean interrupted) {
      intake.Extend();
  }






  

    

    
      
    
}
