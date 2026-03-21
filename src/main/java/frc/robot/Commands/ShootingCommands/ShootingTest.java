package frc.robot.Commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShootingTest extends Command  {
    CommandXboxController controller;
    Shooter shooter;
    Indexer indexer;
    
    public ShootingTest(Indexer indexer,Shooter shooter) {
        this.indexer = indexer;
        this.shooter = shooter;

    }

    @Override
    public void initialize() {
        indexer.setIndexerDutyCycle(1);
        shooter.setFeederVelocity(1);

    }
  
    @Override
    public void execute() {
    }

     @Override
     public void end(boolean interrupted) {
        shooter.setPositionPivot(ShooterConstants.Pivot_HOME);
        shooter.setFeederVelocity(0);
        indexer.setIndexerDutyCycle(0);
        shooter.setShooterVelocity(0);
     }

     @Override
     public boolean isFinished() {
         return false;
     }
    
}
