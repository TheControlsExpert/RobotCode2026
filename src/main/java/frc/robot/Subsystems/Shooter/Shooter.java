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


    private final Alert shooterDisconnectedAlert = new Alert("Shooter Subsystem is Disconnected", AlertType.kError);

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
         shooterDisconnectedAlert.set(!inputs.isConnected);
     }


     public void setOutputPivot(double dutycycle) {
        io.setOutputPivot(dutycycle);
     }

     public void setPositionPivot(double position) {
        io.setPivotPosition(position);
     }

     public void setShooterVelocity(double velocity) {
        io.setVelocityShooter(velocity);
     }

     public void LookupTable_Shooting(double distance) {
  
        io.setPivotPosition(ShootAngleMap.get(distance));
        io.setVelocityShooter(ShootVelocityMap.get(distance));
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
}
