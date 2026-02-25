package frc.robot.Commands.ShootingCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class ResetHood extends Command {

    private final Shooter shooter;
    private final Timer time;



    public ResetHood(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        time = new Timer();
    }



    @Override
    public void initialize() {
        time.restart();
    }


    @Override
    public void execute() {
        shooter.setOutputPivot(-0.1); //moves the pivot at o.5 speed       check constants
    }



    @Override
    public boolean isFinished() {
        boolean pivotIsReset = false;

        if (time.hasElapsed(0.5)) {  
            if (shooter.getPivotVelocity() < 0.01) { //if the pivot can't move any more
                pivotIsReset = true;
            }
        }

        return pivotIsReset;
    }


    @Override
    public void end(boolean interrupted) {
        shooter.setPositionPivot(0);
    }




    
}
