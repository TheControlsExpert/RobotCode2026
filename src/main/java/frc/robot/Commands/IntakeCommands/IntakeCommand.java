package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class IntakeCommand extends Command{
    IntakeSubsystem intake;
    double timeout = 99999;
    Timer timer = new Timer();

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public IntakeCommand(IntakeSubsystem intake, double timeout) {
        this.timeout = timeout;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
        intake.setIntakeDutyCycle(1);
        intake.Extend();
    }

    @Override
    public void end(boolean interrupted) {
        // if (DriverStation.isTeleop()) {
        // intake.setIntakeDutyCycle(0.0);
        // }
        // else {
        intake.setIntakeDutyCycle(0.15);
       // }
    }   

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeout);
    }
}
