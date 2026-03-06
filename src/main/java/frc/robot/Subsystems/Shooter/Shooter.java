package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot.ShootingState;
import frc.robot.Subsystems.Drive.Drive;

public class Shooter extends SubsystemBase {
    
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    //shooting in the hub
    InterpolatingDoubleTreeMap ShootAngleMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap ShootVelocityMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap ShootTOFMap = new InterpolatingDoubleTreeMap();

    //passing
    InterpolatingDoubleTreeMap PassAngleMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap PassVelocityMap = new InterpolatingDoubleTreeMap();

    double phaseDelay = 0.03; 

        //disconnection tracking
    private boolean wasDisconnected_LeftShooter = false;
    private boolean wasDisconnected_RightShooter = false;
    private boolean wasDisconnected_Pivot = false;
    private boolean wasDisconnected_Feeder = false;


    public Shooter(ShooterIO io) {
        this.io = io;

        //put in values here
        ShootAngleMap.put(1.0, 0.5);
        ShootAngleMap.put(2.0, 5.0);
        ShootAngleMap.put(6.0, 10.0);

        ShootVelocityMap.put(1.0, 1500.0);
        ShootVelocityMap.put(4.0, 3500.0);
        ShootVelocityMap.put(6.0, 4000.0);


        PassAngleMap.put(0.0, 1.0);
        PassVelocityMap.put(0.0, 1.0);

        ShootTOFMap.put(0.0, 1.0);
        ShootTOFMap.put(1.0, 1.5);

    }

    
     @Override
     public void periodic() {
         io.updateInputs(inputs);

         if (!wasDisconnected_LeftShooter && !inputs.isConnectedLeftShooter) {
            Robot.reportDisconnection("Left Shooter");
            wasDisconnected_LeftShooter = true;
        }

        if (!wasDisconnected_RightShooter && !inputs.isConnectedRightShooter) {
            Robot.reportDisconnection("Right Shooter");
            wasDisconnected_RightShooter = true;
    
        }

        if (!wasDisconnected_Pivot && !inputs.isConnectedPivot) {
            Robot.reportDisconnection("Shooter Pivot");
            wasDisconnected_Pivot = true;
        }

        if (wasDisconnected_RightShooter && inputs.isConnectedRightShooter) {
            Robot.removeDisconnection("Right Shooter");
            wasDisconnected_RightShooter = false;
        }

        if (wasDisconnected_LeftShooter && inputs.isConnectedLeftShooter) {
            Robot.removeDisconnection("Left Shooter");
            wasDisconnected_LeftShooter = false;
        }

        if (wasDisconnected_Pivot && inputs.isConnectedPivot) {
            Robot.removeDisconnection("Shooter Pivot");
            wasDisconnected_Pivot = false;
        }

        if (!wasDisconnected_Feeder && !inputs.isConnectedFeeder) {
                Robot.reportDisconnection("Feeder");
                wasDisconnected_Feeder = true;
        }
        if (wasDisconnected_Feeder && inputs.isConnectedFeeder) {
                Robot.removeDisconnection("Feeder");
                wasDisconnected_Feeder = false;
        }
     }


     public void setOutputPivot(double dutycycle) { //sets the power into the pivot motor
        io.setOutputPivot(dutycycle);
     }

     public void setPositionPivot(double position) { //tells the pivot what position it is at
        io.setPivotPosition(position);
     }

     public void setShooterVelocity(double velocity) {
        io.setVelocityShooter(velocity);
     }

     public void setOutputShooter(double dutycycle) {
        io.setOutputShooter(dutycycle);
     }

     public double getTOF(double distance) {
        return ShootTOFMap.get(distance);
     }

     public double[] LookupTable_Shooting(Drive drive) {

    // Calculate estimated pose while accounting for phase delay
   
    // ChassisSpeeds robotRelativeVelocity = drive.getRobotRelativeSpeeds();
    // Pose2d beforeEstimatedPose = drive.getEstimatedPosition();
    // Pose2d estimatedPose = beforeEstimatedPose.exp(
        
    //         new Twist2d(
    //             robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
    //             robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
    //             robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target = drive.calculateShootingPosition();
        
    //Pose2d launcherPosition = estimatedPose.transformBy(ShooterConstants.robotToShooter);
    //double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());
    double launcherToTargetDistance = target.getDistance(drive.getEstimatedPosition().getTranslation());


    // Calculate field relative launcher velocity
    // This isn't actually the launcherVelocity given it won't account for angular velocity of robot
    //double launcherVelocityX = drive.getFieldRelativeSpeeds().vxMetersPerSecond;
    //ouble launcherVelocityY = drive.getFieldRelativeSpeeds().vyMetersPerSecond;

    // Account for imparted velocity by robot (launcher) to offset
    // double timeOfFlight = ShootTOFMap.get(launcherToTargetDistance);
    // Pose2d lookaheadPose = launcherPosition;
    // double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    // for (int i = 0; i < 20; i++) {
    //   timeOfFlight = ShootTOFMap.get(lookaheadLauncherToTargetDistance);
    //   double offsetX = launcherVelocityX * timeOfFlight;
    //   double offsetY = launcherVelocityY * timeOfFlight;
    //   lookaheadPose =
    //       new Pose2d(
    //           launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
    //           launcherPosition.getRotation());
    //   lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    // }

    // // Account for launcher being off center
    // Pose2d lookaheadRobotPose =
    //     lookaheadPose.transformBy(ShooterConstants.robotToShooter.inverse());
    // Rotation2d driveAngle = target.minus(lookaheadRobotPose.getTranslation()).getAngle();
    // // Calculate remaining parameters
  
    //     io.setPivotPosition(ShootAngleMap.get(launcherToTargetDistance));
    //     io.setVelocityShooter(ShootVelocityMap.get(launcherToTargetDistance));
    return new double[] {ShootVelocityMap.get(launcherToTargetDistance), ShootAngleMap.get(launcherToTargetDistance)};
     }
    

     public boolean isShooterVelocityLow(double distance) {
          double velocity;
        boolean isShooting = Robot.shootingState.equals(ShootingState.SHOOTING);
        if (isShooting) {
             velocity = ShootVelocityMap.get(distance);
        }
        else {
            velocity = PassVelocityMap.get(distance);
        }

        //at bigger distances, a 100 RPM difference in velocity will probably have a bigger effect than a 100 RPM drop at a smaller difference (since range is proportional to v^2)

        //use difference in v^2 to determine tolerance
        if (isShooting) {
        return Math.abs(Math.pow((inputs.shooterLeftVelocityRPM + inputs.shooterRightVelocityRPM)/2, 2) - Math.pow(velocity, 2)) < ShooterConstants.ShooterVelocity_NoGo_Tolerance;
        }
        else {
        return false; //passing doesn't require as tight of a tolerance since it's not as important to be accurate
        }
     }

    public void LookupTable_Passing(double distance) {
    
        io.setPivotPosition(PassAngleMap.get(distance));
        io.setVelocityShooter(PassVelocityMap.get(distance));
    }

    public boolean isAtShootingVelocity(double distance) {
        double velocity;
        boolean isShooting = Robot.shootingState.equals(ShootingState.SHOOTING);
        if (isShooting) {
             velocity = ShootVelocityMap.get(distance);
        }
        else {
            velocity = PassVelocityMap.get(distance);
        }

        return Math.abs((inputs.shooterLeftVelocityRPM + inputs.shooterRightVelocityRPM) / 2 - velocity) < ShooterConstants.ShooterVelocityTolerance;
    }

    public boolean isAtPivotPosition(double distance) {
        double position;
        boolean isShooting = Robot.shootingState.equals(ShootingState.SHOOTING);
        if (isShooting) {
            position = ShootAngleMap.get(distance);
        }
        else {
            position = PassAngleMap.get(distance);
        }

        return Math.abs(inputs.shooterPivotEncoderRotations - position) < ShooterConstants.ShooterPivotTolerance;
    }

    public void setFeederVelocity(double velocity) {
        io.setFeederVelocity(velocity);
     }

     public double getPivotVelocity() {
        return io.getPivotVelocity();
     }

     
}
