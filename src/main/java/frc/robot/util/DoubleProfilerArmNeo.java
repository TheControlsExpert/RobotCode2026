package frc.robot.util;



import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleProfilerArmNeo {
    private ProfiledPIDController controller;
    double initTime = 0;
    SparkFlex motor;
    double prevvsetpoint = 0;
    boolean hasReset = false;
    double lastResetTime = -10000000;

    public DoubleProfilerArmNeo(ProfiledPIDController controller, SparkFlex motor) {
        this.controller = controller;
        this.motor = motor;
        
    }

    public void initiate() {
        controller.reset(motor.getEncoder().getPosition(), 0);
        initTime = Timer.getFPGATimestamp();
        hasReset = false;
        
    }

    // public boolean shouldUseBiggerkV( double setpoint) {
    //     return 
    // }


    



    public double calculate(double setpoint) {
        if (DriverStation.isEnabled()) {
        SmartDashboard.putBoolean("is profiler being called", true);
        if (Timer.getFPGATimestamp() - lastResetTime > 0.5) {
            initiate();
        }
        lastResetTime  = Timer.getFPGATimestamp();

        if (Timer.getFPGATimestamp() - initTime > 4 && Math.abs(motor.getEncoder().getPosition() - setpoint) > 1) {
            initiate();
        }

        
        if (prevvsetpoint != setpoint) {
           
           
            prevvsetpoint = setpoint;
            initTime = Timer.getFPGATimestamp();
        hasReset = false;
            

        }

        if (Timer.getFPGATimestamp() - initTime > 0.5 && Math.abs(motor.getEncoder().getVelocity()) < 0.25 * 60  && !hasReset) {
            controller.reset(motor.getEncoder().getPosition(), 0);
            hasReset = true;
           
        }

    
        return controller.calculate(motor.getEncoder().getPosition(), setpoint);
    }

return 0;
    
    }
}
