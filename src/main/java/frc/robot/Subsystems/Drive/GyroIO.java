package frc.robot.Subsystems.Drive;


import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double rollDegrees = 0.0;
    public double pitchDegrees = 0.0;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double odometryaccelXpositions = 0.0;
    public double odometryaccelYpositions = 0.0;
  
   
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void resetGyro(Rotation2d rotation) {}

 
}

