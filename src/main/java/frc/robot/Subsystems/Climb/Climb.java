package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final Alert climbAlert = new Alert("Climb subsystem disconnected!", AlertType.kError);

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        climbAlert.set(!inputs.isConnected);

    }


    public void setClimbDutyCycle(double dutyCycle) {
        io.setClimbDutyCycle(dutyCycle);
    }
}
