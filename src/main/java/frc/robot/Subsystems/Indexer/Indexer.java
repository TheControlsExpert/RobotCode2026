package frc.robot.Subsystems.Indexer;

import java.lang.Character.Subset;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Indexer extends SubsystemBase {
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final IndexerIO io;

    private boolean wasDisconnected_Indexer = false;
   


    public Indexer(IndexerIO io) {
        this.io = io;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);

     //   SmartDashboard.putBoolean("indexer is connected?", inputs.isConnected);
      //  SmartDashboard.putBoolean("is indexer was disconnected", wasDisconnected_Indexer);

        if (!wasDisconnected_Indexer && !inputs.isConnected) {
            Robot.reportDisconnection("Indexer");
            wasDisconnected_Indexer = true;
        }
        if (wasDisconnected_Indexer && inputs.isConnected) {
            Robot.removeDisconnection("Indexer");
            wasDisconnected_Indexer = false;
        }
      
    }

    public void setIndexerDutyCycle(double dutyCycle) {
            io.setIndexerDutyCycle(dutyCycle);
    }
    
}
