package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix6.hardware.TalonFXS;

public class IntakeIO {
    TalonFX intakeMotor = new TalonFX(14);
    TalonFXS pivotMotor = new TalonFXS(15);
    DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
    PositionVoltage pivotPositionVoltage = new PositionVoltage(0);
    StatusSignal<Angle> pivotAngle = pivotMotor.getPosition();
    StatusSignal<AngularVelocity> intakeVel = intakeMotor.getVelocity();
    double target = IntakeConstants.HOME_Position;
    boolean Up = true;
    DigitalInput ReadyToClose1 = new DigitalInput(1);
    DigitalInput ReadyToClose2 = new DigitalInput(2);

    

    //DigitalInput HopperFull = new DigitalInput(2);
double offsetEncoder = 0;
    




    public IntakeIO() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        intakeConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;    

        intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.intakeSupplyCurrentLimit;


        intakeMotor.getConfigurator().apply(intakeConfig);

        TalonFXSConfiguration pivotConfig = new TalonFXSConfiguration();
        pivotConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;

        pivotConfig.Slot0.kP = 0;
       // pivotConfig.Slot0.kG = IntakeConstants.pivot_kG;
        pivotConfig.Slot0.kG = 0;
       // pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        pivotConfig.ExternalFeedback.RotorToSensorRatio = 1;
        pivotConfig.ExternalFeedback.SensorToMechanismRatio = 1;

        pivotConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotMotor.setPosition(IntakeConstants.PivotGearRatio * -1 * (pivotEncoder.get() - IntakeConstants.offset));
        
    }

    @AutoLog
    public static class IntakeIOInputs {
        public boolean isConnectedIntake = false;
        public boolean isConnectedPivot = false;
        public boolean isConnectedPivotEncoder = false;
        public double intakePos = 0.0;


        public double pivotEncoderRotations = 0.0;
        public boolean readyToClose1 = false;
        public boolean readyToClose2 = false;
        public boolean hopperFull = false;
    }

    public void updateInputs(IntakeIOInputs inputs) {
       // SmartDashboard.putNumber("target", target);
        inputs.isConnectedIntake = BaseStatusSignal.refreshAll(intakeVel).equals(com.ctre.phoenix6.StatusCode.OK);
        inputs.isConnectedPivot = BaseStatusSignal.refreshAll(pivotAngle).equals(com.ctre.phoenix6.StatusCode.OK);
        inputs.isConnectedPivotEncoder = pivotEncoder.isConnected();

        inputs.pivotEncoderRotations = pivotEncoder.get();
        inputs.intakePos = pivotAngle.getValueAsDouble();

        inputs.readyToClose1 = ReadyToClose1.get();
        inputs.readyToClose2 = ReadyToClose2.get();
        
       // double flipper = Math.signum(target - pivotAngle.getValueAsDouble());
      //  SmartDashboard.putNumber("pivot feedforward", -5 * (target - pivotEncoder.get()));
        // SmartDashboard.putNumber("gravity feed", Math.cos(offsetEncoder * 2 * Math.PI) * IntakeConstants.pivot_kG * 12);
        // SmartDashboard.putNumber("spring feed", Math.abs(Math.sin(offsetEncoder * 2 * Math.PI)) * IntakeConstants.cf_spring * 12);
       
        // }
        //TO-DO: add voltage limits
        offsetEncoder = pivotEncoder.get() - IntakeConstants.offset;

        if (pivotEncoder.isConnected()) {

        if (!Up) {
        pivotMotor.set(MathUtil.clamp(-0.3, IntakeConstants.pivot_kP_down * (target - pivotEncoder.get()), 0.3));
         SmartDashboard.putNumber("feedforward", MathUtil.clamp(-0.3, IntakeConstants.pivot_kP_down * (target - pivotEncoder.get()), 0.3));
        }

        else {
        pivotMotor.set(MathUtil.clamp(-0.4, -2 * (target - pivotEncoder.get()), 0.4));
         SmartDashboard.putNumber("feedforward", MathUtil.clamp(-0.4, -2 * (target - pivotEncoder.get()), 0.4));
        }

        if (Up && pivotEncoder.get() > 0.60 ) {
            pivotMotor.set(0);
            SmartDashboard.putNumber("feedforward", 0);
        }

        if (!Up && pivotEncoder.get() < 0.24) {
            pivotMotor.set(0);
            SmartDashboard.putNumber("feedforward", 0);
        }
    }
    else {
        pivotMotor.set(0);
    }
       // SmartDashboard.putNumber("pivot voltage", pivotMotor.getDutyCycle().getValueAsDouble());
        //inputs.readyToClose = ReadyToClose.get();
        //inputs.hopperFull = !HopperFull.get();
    }


    public void setIntakeDutyCycle(double dutyCycle) {
        intakeMotor.setControl(new DutyCycleOut(dutyCycle));
    }

    public void setPosition(double position) {
        SmartDashboard.putNumber("intake position", position);
        target = position;
       // pivotMotor.setControl(pivotPositionVoltage.withPosition(position));
    }

   
    public void resetPosition() {
        //pivotMotor.setPosition(IntakeConstants.PivotGearRatio * (pivotEncoder.get() - IntakeConstants.offset));
    }

    // public enum IntakeStates {
    //     INTAKING,
    //     HOME,
    //     SHUFFLE_UP
    // }







    }
  

