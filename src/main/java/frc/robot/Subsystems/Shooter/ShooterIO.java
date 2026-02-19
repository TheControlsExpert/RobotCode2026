package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class ShooterIO {
    TalonFX shooterLeft = new TalonFX(18);
    TalonFX shooterRight = new TalonFX(19);
    TalonFXS shooterPivot = new TalonFXS(20);

    VelocityVoltage shooterLeftVoltage = new VelocityVoltage(0);
    VelocityVoltage shooterRightVoltage = new VelocityVoltage(0);
    PositionVoltage shooterPivotVoltage = new PositionVoltage(0);


    public ShooterIO() {
        TalonFXConfiguration shooterL = new TalonFXConfiguration();

        shooterL.Slot0.kV = ShooterConstants.shooterL_kV;
        shooterL.Slot0.kS = ShooterConstants.shooterL_kS;
        shooterL.Slot0.kP = ShooterConstants.shooterL_kP;
        shooterL.Slot0.kD = ShooterConstants.shooterL_kD;

        shooterL.MotorOutput.Inverted = ShooterConstants.shooterL_inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        shooterL.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterL.CurrentLimits.SupplyCurrentLimit = ShooterConstants.supplyCurrentLimit;
        shooterL.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.supplyCurrentLowerLimit;
        shooterL.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.supplyCurrentLowerLimit_time;

        shooterLeft.getConfigurator().apply(shooterL);

        TalonFXConfiguration shooterR = new TalonFXConfiguration();
        shooterR.Slot0.kV = ShooterConstants.shooterR_kV;
        shooterR.Slot0.kS = ShooterConstants.shooterR_kS;
        shooterR.Slot0.kP = ShooterConstants.shooterR_kP;
        shooterR.Slot0.kD = ShooterConstants.shooterR_kD;

        shooterR.MotorOutput.Inverted = ShooterConstants.shooterR_inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        shooterR.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterR.CurrentLimits.SupplyCurrentLimit = ShooterConstants.supplyCurrentLimit;
        shooterR.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.supplyCurrentLowerLimit;
        shooterR.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.supplyCurrentLowerLimit_time;

        shooterRight.getConfigurator().apply(shooterR);

        TalonFXSConfiguration pivot = new TalonFXSConfiguration();
        
        pivot.Slot0.kP = ShooterConstants.shooterPivot_kP;
        pivot.Slot0.kD = ShooterConstants.shooterPivot_kD;

        pivot.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //note inverted value doesn't matter here, since we will be receving setpoint encoder positions from interpolating tree map
        pivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooterPivot.getConfigurator().apply(pivot);
    
    }

    @AutoLog
    public class ShooterIOInputs {
        
         
    }
    
}
