package frc.robot.Subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    SparkFlex motor2 = new SparkFlex(1, MotorType.kBrushless);

    public Intake() {

    }

    public void setSpeed(double speed) {
        motor2.set(speed);
                                                       
    }
    
}
