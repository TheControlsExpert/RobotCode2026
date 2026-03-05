package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class ShooterIO {

    TalonFX shooterLeft = new TalonFX(18);
    TalonFX shooterRight = new TalonFX(19);
    TalonFXS shooterPivot = new TalonFXS(20);
    TalonFX feeder = new TalonFX(17);

    VelocityVoltage shooterLeftVoltage = new VelocityVoltage(0);
    VelocityVoltage shooterRightVoltage = new VelocityVoltage(0);
    PositionVoltage shooterPivotVoltage = new PositionVoltage(0);
    DutyCycleOut shooterPivotDutyCycle = new DutyCycleOut(0);
    DutyCycleOut feederDutyCycle = new DutyCycleOut(0);

    StatusSignal<AngularVelocity> shooterLeftVelocity;
    StatusSignal<AngularVelocity> shooterRightVelocity;
    StatusSignal<Angle> shooterPivotPosition;
    StatusSignal<AngularVelocity> feederVelocity;
    
  
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

        shooterL.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorCurrentLimit;
        

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

        shooterR.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorCurrentLimit;

        shooterRight.getConfigurator().apply(shooterR);

        TalonFXSConfiguration pivot = new TalonFXSConfiguration();
        
        pivot.Slot0.kP = ShooterConstants.shooterPivot_kP;
        pivot.Slot0.kD = ShooterConstants.shooterPivot_kD;

        //note inverted value doesn't matter here, since we will be receving setpoint encoder positions from interpolating tree map
        pivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooterPivot.getConfigurator().apply(pivot);

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.feederSupplyCurrentLimit;


        feeder.getConfigurator().apply(feederConfig);

        shooterLeftVelocity = shooterLeft.getVelocity();
        shooterRightVelocity = shooterRight.getVelocity();
        shooterPivotPosition = shooterPivot.getPosition();
        feederVelocity = feeder.getVelocity();


    }

    @AutoLog
    public static class ShooterIOInputs {
        public boolean isConnectedLeftShooter = false;
        public boolean isConnectedRightShooter = false;
        public boolean isConnectedPivot = false;
        public boolean isConnectedFeeder = false;

        public double shooterLeftVelocityRPM = 0.0;
        public double shooterRightVelocityRPM = 0.0;
        public double shooterPivotEncoderRotations = 0.0;
        public double feederVelocityRPM = 0.0;
         
    }


    public void updateInputs(ShooterIOInputs inputs) {
        inputs.isConnectedLeftShooter = BaseStatusSignal.refreshAll(shooterLeftVelocity).equals(StatusCode.OK);
        inputs.isConnectedRightShooter = BaseStatusSignal.refreshAll(shooterRightVelocity).equals(StatusCode.OK);
        inputs.isConnectedPivot = BaseStatusSignal.refreshAll(shooterPivotPosition).equals(StatusCode.OK);
        inputs.isConnectedFeeder = BaseStatusSignal.refreshAll(feederVelocity).equals(StatusCode.OK);
        
        inputs.shooterLeftVelocityRPM = shooterLeftVelocity.getValue().in(RPM);
        inputs.shooterRightVelocityRPM = shooterRightVelocity.getValue().in(RPM);
        inputs.shooterPivotEncoderRotations = shooterPivotPosition.getValue().in(Rotations);    
        inputs.feederVelocityRPM = feederVelocity.getValue().in(RPM);
  }

  public void setOutputPivot(double dutycycle) {
    shooterPivot.setControl(shooterPivotDutyCycle.withOutput(dutycycle));
  }

  public void setPivotPosition(double position) {
    SmartDashboard.putNumber("shooter pivot position", position);
    shooterPivot.setControl(shooterPivotVoltage.withPosition(position));
  }

  public void setFeederVelocity(double velocity) {
    SmartDashboard.putNumber("feeder velocity", velocity);
    feeder.setControl(feederDutyCycle.withOutput(velocity));
  }

  public void setVelocityShooter(double velocity) {
    SmartDashboard.putNumber("shooter velocity", velocity);
    shooterLeft.setControl(shooterLeftVoltage.withVelocity(velocity));
    shooterRight.setControl(shooterRightVoltage.withVelocity(velocity));
  }

  public void setOutputShooter(double dutycycle) {
    shooterLeft.setControl(new DutyCycleOut(dutycycle));
    shooterRight.setControl(new DutyCycleOut(dutycycle));
  }

  public double getPivotVelocity() {
    return shooterPivot.getVelocity().getValue().in(RPM);
  }

}  
