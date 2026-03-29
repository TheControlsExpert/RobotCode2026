package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.IndexerConstants;

public class IndexerIO  {
    TalonFX indexerMotor = new TalonFX(16);
    StatusSignal<Current> indexerSupplyCurrent = indexerMotor.getSupplyCurrent();

        public IndexerIO() {
            TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
            indexerConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
            indexerConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;

            indexerConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.supplyCurrentLimit;

            indexerMotor.getConfigurator().apply(indexerConfig);

    
        }
    
        @AutoLog
        public static class IndexerIOInputs {
            public boolean isConnected = true;
            public double indexerSupplyCurrent = 0.0;
            
        }
        public void updateInputs(IndexerIOInputs inputs) {
            inputs.isConnected = com.ctre.phoenix6.BaseStatusSignal.refreshAll(indexerSupplyCurrent).equals(com.ctre.phoenix6.StatusCode.OK);
            inputs.indexerSupplyCurrent = indexerSupplyCurrent.getValueAsDouble();
    
        }
    
        public void setIndexerDutyCycle(double dutyCycle) {
            indexerMotor.setControl(new DutyCycleOut(dutyCycle).withEnableFOC(true));
    
        }
    
}
