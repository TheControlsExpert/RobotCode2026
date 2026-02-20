package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
    
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    Alert intakeDisconnectedAlert = new Alert("Intake subsystem is disconnected!", Alert.AlertType.kError);

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        intakeDisconnectedAlert.set(!inputs.isConnected);
    }

    public void setIntakeDutyCycle(double dutyCycle) {
        io.setIntakeDutyCycle(dutyCycle);
    }

    public void setPivotPosition(double position) {
        io.setPosition(position);
    }

    public void resetPivotPosition() {
        io.resetPosition();
 }

}