package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;

public class Jam extends Command {
    private final Indexer indexer;
    private final Shooter shooter;
    private final Timer timer;
    private final double timeLimit;


    public Jam(Indexer indexer, Shooter shooter, Double timeLimit) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.timeLimit = timeLimit;
        timer = new Timer();
        addRequirements(indexer, shooter);
    }

    public Jam(Indexer indexer, Shooter shooter) {
        this.indexer = indexer;
        this.shooter = shooter;
        timer = new Timer();
        timeLimit = 9999; // a time that will never be reached
        addRequirements(indexer, shooter);
    }


    @Override
    public void initialize() {
        timer.restart();
        indexer.setIndexerDutyCycle(-0.35);
        shooter.setFeederVelocity(-0.35);
      //  intake.setIntakeDutyCycle(-0.2);
 
    }


    public boolean isFinished() {
        return timer.hasElapsed(timeLimit);
    }

    
    @Override
    public void end(boolean interrupted) {
        indexer.setIndexerDutyCycle(0);
        shooter.setFeederVelocity(0);
        shooter.setShooterVelocity(0);
        shooter.setPositionPivot(ShooterConstants.Pivot_HOME);

        shooter.isShooting = false;
        RobotContainer.isShooting = false;
      //  intake.setIntakeDutyCycle(0);
    }
    
}
