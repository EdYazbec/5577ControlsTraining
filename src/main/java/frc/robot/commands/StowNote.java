// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class StowNote extends Command {
    // first, declare the things that will exist for this command
    private Indexer indexerSubsystem;
    private double outputValue;

    /** Creates a new StowNote. */
    public StowNote(Indexer indexerSubsystem, double outputValue) {
        this.indexerSubsystem = indexerSubsystem;
        this.outputValue = outputValue;

        // Use addRequirements() here to declare subsystem dependencies.
        // this command will need to yoink control of the indexer
        addRequirements(indexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexerSubsystem.setMotorOutput(outputValue);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.setMotorOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return indexerSubsystem.noteStowed();
    }
}
