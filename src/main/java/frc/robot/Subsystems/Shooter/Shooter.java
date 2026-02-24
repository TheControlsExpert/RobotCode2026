package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot.ShootingState;

public class Shooter extends SubsystemBase {
    
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    //shooting in the hub
    InterpolatingDoubleTreeMap ShootAngleMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap ShootVelocityMap = new InterpolatingDoubleTreeMap();

    //passing
    InterpolatingDoubleTreeMap PassAngleMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap PassVelocityMap = new InterpolatingDoubleTreeMap();

        //disconnection tracking
    private boolean wasDisconnected_LeftShooter = false;
    private boolean wasDisconnected_RightShooter = false;
    private boolean wasDisconnected_Pivot = false;
    private boolean wasDisconnected_Feeder = false;


    public Shooter(ShooterIO io) {
        this.io = io;

        //put in values here
        ShootAngleMap.put(0.0, 1.0);
        ShootVelocityMap.put(0.0, 1.0);
        PassAngleMap.put(0.0, 1.0);
        PassVelocityMap.put(0.0, 1.0);
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

     public void LookupTable_Shooting(double distance) {
  
        io.setPivotPosition(ShootAngleMap.get(distance));
        io.setVelocityShooter(ShootVelocityMap.get(distance));
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
