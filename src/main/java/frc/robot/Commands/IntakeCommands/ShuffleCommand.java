package frc.robot.Commands.IntakeCommands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class ShuffleCommand {
    IntakeSubsystem intake;

    public ShuffleCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

   public Command getShuffleCommand() {
    return new InstantCommand(() -> {intake.Retract();}, intake)
               .andThen(new WaitCommand(0.7))
               .andThen(new InstantCommand(() -> {intake.Extend();}, intake))
               .andThen(new WaitCommand(0.5))
               .andThen(new InstantCommand(() -> {intake.Retract();}, intake))
               .andThen(new WaitCommand(0.7))
               .andThen(new InstantCommand(() -> {intake.Extend();}, intake));

   }
    
    


    
}
