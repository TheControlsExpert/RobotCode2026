package frc.robot.Subsystems.Intake;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.Intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
    
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    ArrayList<Double> IntakeFullHistory = new ArrayList<>();
    ArrayList<Double> ReadyToCloseHistory = new ArrayList<>();
    //public boolean beep = false;

    double averageIntakeFull = 0;   
    double averageReadyToClose = 0;
    public boolean is_busy = false;

    //disconnection tracking
    private boolean wasDisconnected_Intake = false;
    private boolean wasDisconnected_Pivot = false;
  


    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
       // setPivotPosition(0.725);
       // io.Up = true;
        SmartDashboard.putBoolean("Hopper Full?", isHopperFull());
        SmartDashboard.putNumber("Encoderabs", inputs.pivotEncoderRotations);
        SmartDashboard.putNumber("encoder intake", inputs.intakePos);

        if (IntakeFullHistory.size() > 20) {
            IntakeFullHistory.remove(0);
            ReadyToCloseHistory.remove(0);
        }

        IntakeFullHistory.add(inputs.hopperFull ? 1.0 : 0.0);
        ReadyToCloseHistory.add(inputs.readyToClose ? 1.0 : 0.0);

        averageIntakeFull = IntakeFullHistory.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        averageReadyToClose = ReadyToCloseHistory.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

        if (DriverStation.isDisabled()) {
            if (!inputs.isConnectedIntake && !wasDisconnected_Intake) {
                Robot.reportDisconnection("Intake Motor");
                wasDisconnected_Intake = true;
                

            }
            
            if (wasDisconnected_Intake && inputs.isConnectedIntake) {
                Robot.removeDisconnection("Intake Motor");
                wasDisconnected_Intake = false;
            }

            if (!inputs.isConnectedPivot && !wasDisconnected_Pivot) {
                    Robot.reportDisconnection("Intake Pivot");
                    wasDisconnected_Pivot = true;
            }

            if (wasDisconnected_Pivot && inputs.isConnectedPivot) {
                    Robot.removeDisconnection("Intake Pivot");
                    wasDisconnected_Pivot = false;
            }
        }
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

   public void Retract() {
        io.setPosition(IntakeConstants.HOME_Position);
        io.Up = true;
    }

    public void Extend() {
        io.setPosition(IntakeConstants.INTAKING_Position);
        io.Up = false;
    }

    public void Shuffle() {
        io.setPosition(IntakeConstants.SHUFFLE_UP_POSITION);
        io.Up = true;
    }

    public void retractBump() {
        io.setPosition(IntakeConstants.BUMPING_Position);
    }

    public boolean isReadyToClose() {
     //  return beep;
       return averageReadyToClose > 0.75;
    }

    public boolean isHopperFull() {
        return averageIntakeFull > 0.9;
    }


}