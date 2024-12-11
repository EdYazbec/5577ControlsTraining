// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinShooter extends Command {
    /** Creates a new SpinShooter. */
    private Shooter shooterSubsystem;
    private double outputValue;

    public SpinShooter(Shooter shooterSubsystem, double outputValue) {
        this.shooterSubsystem = shooterSubsystem;
        this.outputValue = outputValue;

        // Use addRequirements() here to declare subsystem dependencies.
        // this command will need to yoink control of the shooter
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSubsystem.setSpeed(outputValue);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // when this commmand ends, we should turn off the motors
        shooterSubsystem.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // this command does not have an end condition, it should be terminated by the scheduler
        return false;
    }
}
