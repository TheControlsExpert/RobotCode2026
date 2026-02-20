package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.IntakeConstants;

public class IntakeIO {
    TalonFX intakeMotor = new TalonFX(14);
    TalonFX pivotMotor = new TalonFX(15);
    DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
    PositionVoltage pivotPositionVoltage = new PositionVoltage(0);
    StatusSignal<Angle> pivotAngle = pivotMotor.getPosition();
    StatusSignal<AngularVelocity> intakeVel = intakeMotor.getVelocity();
    




    public IntakeIO() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        intakeConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;    

        intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.intakeSupplyCurrentLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.intakeSupplyCurrentLowerLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.intakeSupplyCurrentLowerLimit_time;

        intakeMotor.getConfigurator().apply(intakeConfig);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

        pivotConfig.Slot0.kP = IntakeConstants.pivot_kP;
        pivotConfig.Slot0.kG = IntakeConstants.pivot_kG;

        pivotConfig.Feedback.RotorToSensorRatio = 1;
        pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.PivotGearRatio;

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(IntakeConstants.PivotGearRatio * (pivotEncoder.get() - IntakeConstants.offset));
        

    }

    @AutoLog
    public static class IntakeIOInputs {
        public boolean isConnected = true;
        public double pivotEncoderRotations = 0.0;
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(pivotAngle, intakeVel).equals(com.ctre.phoenix6.StatusCode.OK);
        inputs.pivotEncoderRotations = pivotAngle.getValue().magnitude();
    }


    public void setIntakeDutyCycle(double dutyCycle) {
        intakeMotor.setControl(new DutyCycleOut(dutyCycle));
    }

    public void setPosition(double position) {
        pivotMotor.setControl(pivotPositionVoltage.withPosition(position));
    }

    public void resetPosition() {
        pivotMotor.setPosition(IntakeConstants.PivotGearRatio * (pivotEncoder.get() - IntakeConstants.offset));
    }



    }
  

