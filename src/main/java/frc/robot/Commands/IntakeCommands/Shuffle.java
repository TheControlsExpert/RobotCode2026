package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;

public class Shuffle extends Command {
    private final Indexer indexer;
    private final Shooter shooter;

    public Shuffle(Indexer indexer, Shooter shooter) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(indexer, shooter);
    }


    @Override
    public void initialize() {
        indexer.setIndexerDutyCycle(-0.4);
        shooter.setFeederVelocity(-0.8);
 
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setIndexerDutyCycle(0);
        shooter.setFeederVelocity(0);
    }
    
}
