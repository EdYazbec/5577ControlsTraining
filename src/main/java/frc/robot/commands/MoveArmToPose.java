// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToPose extends Command {
    Arm arm;
    double setpoint;

    /** Creates a new MoveArmToPose. */
    public MoveArmToPose(Arm armSubsystem, double setpoint) {
        this.arm = armSubsystem;
        this.setpoint = setpoint;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setArmPos(setpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return arm.isAtSetpoint();
    }
}
