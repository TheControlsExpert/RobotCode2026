package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private boolean climbGoalUp;
    
     //disconnection tracking
     private boolean wasDisconnected = false;


    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (!wasDisconnected && !inputs.isConnected) {
            Robot.reportDisconnection("Climb");
            wasDisconnected = true;
        }
        if (wasDisconnected && inputs.isConnected) {
            Robot.removeDisconnection("Climb");
            wasDisconnected = false;
        }

    }


    public void setClimbDutyCycle(double dutyCycle) {
        io.setClimbDutyCycle(dutyCycle);
    }


    public double getEncoderValue() {
        return io.getEncoderValue();
    }

    public void setClimbGoalUp(boolean isClimbGoalUp) {
        climbGoalUp = isClimbGoalUp;
    }

    public boolean isClimbGoalUp() {
        return climbGoalUp;
    }
}
