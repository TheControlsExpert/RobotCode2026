package frc.robot.Subsystems.Indexer;

import java.lang.Character.Subset;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final IndexerIO io;
    Alert indexerDisconnectedAlert = new Alert("Indexer subsystem is disconnected!", Alert.AlertType.kError);


    public Indexer(IndexerIO io) {
        this.io = io;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        indexerDisconnectedAlert.set(!inputs.isConnected);
       
    }

    public void setIndexerDutyCycle(double dutyCycle) {
            io.setIndexerDutyCycle(dutyCycle);
    }
    
}
