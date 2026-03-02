package frc.robot.Subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ClimbConstants;


public class ClimbIO {
    TalonFX climbMotor = new TalonFX(21);
    DutyCycleOut climbDutyCycleOut = new DutyCycleOut(0);

    StatusSignal<Angle> climbEncoderPosition = climbMotor.getPosition();


    public ClimbIO() {
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();

        climbConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        climbConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

        climbConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.supplyCurrentLimit;
        climbConfig.CurrentLimits.SupplyCurrentLowerLimit = ClimbConstants.supplyCurrentLowerLimit;
        climbConfig.CurrentLimits.SupplyCurrentLowerTime = ClimbConstants.supplyCurrentLowerLimit_time;

        climbMotor.getConfigurator().apply(climbConfig);
    }

    @AutoLog
    public static class ClimbIOInputs {
        public boolean isConnected = true;
        public double encoderPosition = 0.0;

    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(climbEncoderPosition).equals(StatusCode.OK);
        inputs.encoderPosition = climbEncoderPosition.getValueAsDouble();
    }

    public void setClimbDutyCycle(double dutyCycle) {
        climbMotor.setControl(climbDutyCycleOut.withOutput(dutyCycle));
    }

    public double getEncoderValue() {
        return climbMotor.getPosition().getValueAsDouble();
    }
   
}
