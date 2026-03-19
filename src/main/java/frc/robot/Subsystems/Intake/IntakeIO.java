package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.Rotations;

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
import frc.robot.Commands.IntakeCommands.IntakeCommand;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix6.hardware.TalonFXS;

public class IntakeIO {
    TalonFX intakeMotor = new TalonFX(14);
    TalonFXS pivotMotor = new TalonFXS(15);
    DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0, 1, 0);
    PositionVoltage pivotPositionVoltage = new PositionVoltage(0);
    StatusSignal<Angle> pivotAngle = pivotMotor.getPosition();
    StatusSignal<AngularVelocity> intakeVel = intakeMotor.getVelocity();
    double target = IntakeConstants.HOME_Position;
    boolean Up = true;
    DigitalInput ReadyToClose1 = new DigitalInput(2);
    //DigitalInput ReadyToClose2 = new DigitalInput(2);

    boolean resetCorrectly = false;


    

    //DigitalInput HopperFull = new DigitalInput(2);

    




    public IntakeIO() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        intakeConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;    

        intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.intakeSupplyCurrentLimit;


        intakeMotor.getConfigurator().apply(intakeConfig);

        TalonFXSConfiguration pivotConfig = new TalonFXSConfiguration();
        pivotConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
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
        if (pivotEncoder.isConnected()) {
        resetCorrectly = true;  
        }
        pivotMotor.setPosition(IntakeConstants.PivotGearRatio * (pivotEncoder.get()));
        
    }

    @AutoLog
    public static class IntakeIOInputs {
        public boolean isConnectedIntake = false;
        public boolean isConnectedPivot = false;
        public boolean isConnectedPivotEncoder = false;
        public double intakePos = 0.0;


        public double pivotEncoderRotations = 0.0;
        public boolean readyToClose1 = false;
      //  public boolean readyToClose2 = false;
        public boolean hopperFull = false;
    }

    public void updateInputs(IntakeIOInputs inputs) {
        SmartDashboard.putBoolean("intake IR", ReadyToClose1.get());
       SmartDashboard.putNumber("target", target);
       SmartDashboard.putBoolean("up", Up);
       SmartDashboard.putBoolean("intake encoder connected", pivotEncoder.isConnected());
       SmartDashboard.putNumber("intake encoder", pivotEncoder.get());
       SmartDashboard.putNumber("intake angle", pivotAngle.getValue().in(Rotations));
        inputs.isConnectedIntake = BaseStatusSignal.refreshAll(intakeVel).equals(com.ctre.phoenix6.StatusCode.OK);
        inputs.isConnectedPivot = BaseStatusSignal.refreshAll(pivotAngle).equals(com.ctre.phoenix6.StatusCode.OK);
        inputs.isConnectedPivotEncoder = pivotEncoder.isConnected();

        inputs.pivotEncoderRotations = pivotEncoder.get();
        inputs.intakePos = pivotAngle.getValueAsDouble();

        inputs.readyToClose1 = ReadyToClose1.get();
    //   //  inputs.readyToClose2 = ReadyToClose2.get();


        if (pivotEncoder.isConnected() && !resetCorrectly) {
            pivotMotor.setPosition(IntakeConstants.PivotGearRatio * (pivotEncoder.get()));
            resetCorrectly = true;
        }   
        
       // else if (!pivotEncoder.isConnected()) {
       

       if (pivotEncoder.isConnected()) {
         if (target > pivotEncoder.get()) {
            Up = true;
        }

        else {
            Up = false;
        }
      //  double adjustedEncoder = pivotEncoder.get() - IntakeConstants.offset;
         if (Up && pivotEncoder.get() > IntakeConstants.MAX_ENCODER_VAL) {
            pivotMotor.set(0);
            SmartDashboard.putNumber("feedforward", 0);
        }

        else if (!Up && pivotEncoder.get() < IntakeConstants.MIN_ENCODER_VAL) {
            pivotMotor.set(0);
            SmartDashboard.putNumber("feedforward", 0);
        }
        
        else {

           
        if (!Up) {
            //set voltage limits + use kP for going down +     
        
        double clampedVal =  IntakeConstants.pivot_kP_down * (target - pivotEncoder.get());   
        if ( IntakeConstants.pivot_kP_down * (target - pivotEncoder.get()) > 0.3) {
            clampedVal = 0.3;
        }

        else if ( IntakeConstants.pivot_kP_down * (target - pivotEncoder.get()) < -0.3) {
            clampedVal = -0.3;
         
        }  
        
      //  pivotMotor.set(clampedVal);
         SmartDashboard.putNumber("feedforward", clampedVal);
        }

        else {
        SmartDashboard.putNumber("target", target);
        double clampedVal =  IntakeConstants.pivot_kP_up * (target - pivotEncoder.get());
        
       
        if ( IntakeConstants.pivot_kP_up * (target - pivotEncoder.get()) > 0.4) {
            clampedVal = 0.4;

        }

        else if ( IntakeConstants.pivot_kP_up * (target - pivotEncoder.get()) < -0.4) {
            clampedVal = -0.4;
         
        }
        SmartDashboard.putNumber("feedforward", clampedVal);
      //   pivotMotor.set(clampedVal);

        }
        }}


    else if (!pivotEncoder.isConnected() && resetCorrectly) {
        // double adjustedTarget = IntakeConstants.PivotGearRatio * (target);

        // double adjustedLimit_MAX = IntakeConstants.PivotGearRatio * (IntakeConstants.MAX_ENCODER_VAL);
        // double adjustedLimit_MIN = IntakeConstants.PivotGearRatio * (IntakeConstants.MIN_ENCODER_VAL);

        // double adjustedkP_down = IntakeConstants.pivot_kP_down * IntakeConstants.PivotGearRatio;
        // double adjustedkP_up = IntakeConstants.pivot_kP_up * IntakeConstants.PivotGearRatio; 



        // if (adjustedTarget > inputs.intakePos) {
        //     Up = true;
        // }

        // else {
        //     Up = false;
        // }  
        
        // if (!Up) {    
        // double clampedVal =  adjustedkP_down * (target - inputs.intakePos);

        // if ( adjustedkP_down * (target - inputs.intakePos) > 0.3) {
        //     clampedVal = 0.3;

        // }

        // else if ( adjustedkP_down * (target - inputs.intakePos) < -0.3) {
        //     clampedVal = -0.3;
         
        // }
        // pivotMotor.set(clampedVal);
        //  SmartDashboard.putNumber("feedforward", clampedVal);
        // }

        // else {
        // double clampedVal =  adjustedkP_up * (target - inputs.intakePos);
        // if ( adjustedkP_up * (target - inputs.intakePos) > 0.4) {
        //     clampedVal = 0.4;

        // }

        // else if ( adjustedkP_up * (target - inputs.intakePos) < -0.4) {
        //     clampedVal = -0.4;
         
        // }
        // pivotMotor.set(clampedVal);
        //  SmartDashboard.putNumber("feedforward", clampedVal);
        // }

        // if (Up && inputs.intakePos > adjustedLimit_MAX) {
        //     pivotMotor.set(0);
        //     SmartDashboard.putNumber("feedforward", 0);
        // }

        // if (!Up && inputs.intakePos < adjustedLimit_MIN) {
        //     pivotMotor.set(0);
        //     SmartDashboard.putNumber("feedforward", 0);
        // }  
    }

    else {
        pivotMotor.set(0);
        SmartDashboard.putNumber("feedforward", 0);
    }
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
  

