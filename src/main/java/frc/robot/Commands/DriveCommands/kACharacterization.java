package frc.robot.Commands.DriveCommands;

import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Drive.Drive;

public class kACharacterization  {
    
    private static final double FF_START_DELAY = 2;
    private static final double FF_RAMP_RATE = 0.1;
    
    
        public static Command feedforwardCommand(Drive drive, XboxController controller) { 
            List<Double> velocitySamples = new LinkedList<>();
            List<Double> accelerationSamples = new LinkedList<>();
            List<Double> voltageSamples = new LinkedList<>();
            Timer timer = new Timer();
    
        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),
    
            // Allow modules to orient
            Commands.run(
                    () -> {
                      drive.runCharacterization(0.0);
                    },
                    drive)
                .withTimeout(FF_START_DELAY),
    
            // Start timer
            Commands.runOnce(timer::restart),
    
            // Accelerate and gather data
            Commands.run(
                    () -> {
                      double voltage = 5;
                  drive.runCharacterization(voltage);
                  SmartDashboard.putNumber("accel", drive.getFFCharacterizationAcceleration());
                  if (controller.getXButton() && timer.hasElapsed(0.2) && !timer.hasElapsed(2) && Math.abs(drive.getFFCharacterizationAcceleration()) > 0.5) {
                  
                  velocitySamples.add(Math.abs(drive.getFFCharacterizationVelocity()));
                  accelerationSamples.add(Math.abs(drive.getFFCharacterizationAcceleration()));
                  voltageSamples.add(voltage);
                  }
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = accelerationSamples.size();
                  SmartDashboard.putNumber("n", n);
                  double sumX2 = 0.0;
                
                  double sumXY = 0.0;
                 
                  for (int i = 0; i < n; i++) {
                    sumX2 += Math.pow(accelerationSamples.get(i), 2);
                    sumXY += accelerationSamples.get(i) * (voltageSamples.get(i) - SwerveConstants.driveKS - velocitySamples.get(i) * SwerveConstants.driveKV);
                    
                  }
                  
                   double kA = sumXY / sumX2;

                  SmartDashboard.putNumber("kA", kA);
                  

    }));
}
  }