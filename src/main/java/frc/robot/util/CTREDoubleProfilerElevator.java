package frc.robot.util;



import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CTREDoubleProfilerElevator {
    
    double initTime = 0;
    TalonFX motor;
    double prevvsetpoint = 0;
    boolean hasReset = false;
    double lastResetTime = -10000000;

    public CTREDoubleProfilerElevator(TalonFX motor) {
       
        this.motor = motor;
        
    }

    public void initiate() {
        initTime = Timer.getFPGATimestamp();
        hasReset = false;
        
    }

    // public boolean shouldUseBiggerkV( double setpoint) {
    //     return 
    // }


    



    public void runProfile(double setpoint, MotionMagicVoltage setter) {
        SmartDashboard.putBoolean("is profiler being called", true);
        if (Timer.getFPGATimestamp() - lastResetTime > 0.5) {
            initiate();
        }
        lastResetTime  = Timer.getFPGATimestamp();

        
        if (prevvsetpoint != setpoint) {
           
           
        prevvsetpoint = setpoint;
        initiate();
            

        }
      
            motor.setControl(setter.withPosition(setpoint));
       


        if (Timer.getFPGATimestamp() - initTime > 5 && Math.abs(motor.getPosition().getValueAsDouble() - setpoint) > 1.5) {
            initiate();
        }
    }
    
}
