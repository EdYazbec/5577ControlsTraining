// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MoveArmToPose;
import frc.robot.commands.SetIndexerSpeed;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.StowNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    /* Swerve specific setup */
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 0.5 * Math.PI;               // 1/2 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;     // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // Drive with field-centric
      .withDeadband(MaxSpeed * 0.1)                     // Add a 10% deadband to driving
      .withRotationalDeadband(MaxAngularRate * 0.1)     // Add a 10% deadband to turning
      .withDriveRequestType(DriveRequestType.Velocity); // driving in velocity mode
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Other subsystems */
    private final Shooter shooter = new Shooter();
    private final Indexer indexer = new Indexer();
    public final Arm arm = new Arm();

    /* Commands */
    // its good practice to only create the commands once. this makes java's memory management more happy
    // were using the same command to set the shooter speed, but just passing different values to it
    // this is allows the command scheduler to manage priority
    
    /* Shooter commands */
    SpinShooter setShooterSpeedForSpeaker = new SpinShooter(shooter, Constants.ArmProfile.kShooterDefaultOutput);
    SpinShooter setShooterSpeedForAmp = new SpinShooter(shooter, Constants.ArmProfile.kShooterAmpOutput);

    /* Indexer Commands */
    StowNote stowNote = new StowNote(indexer, Constants.ArmProfile.kIndexerDefaultOutput);
    // there seems to be a minor issue (robot crashes) when registering the same command instance to multiple command sequences 
    // through the use of command.andThen(). So for now, ill make each instance of the command below in line when they are used. 
    // SetIndexerSpeed indexerTransferIndexerSpeed = new SetIndexerSpeed(indexer, Constants.ArmProfile.kIndexerDefaultOutput);

    /* Arm Commands */
    MoveArmToPose armToSpeaker = new MoveArmToPose(arm, Constants.ArmProfile.kArmSpeakerPos);
    MoveArmToPose armToAmp = new MoveArmToPose(arm, Constants.ArmProfile.kArmAmpPos);
    MoveArmToPose armToZero = new MoveArmToPose(arm, Constants.ArmProfile.kArmInitialPos);

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () -> drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)  // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)             // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.y().onTrue(armToSpeaker);
    joystick.y().whileTrue(setShooterSpeedForSpeaker.alongWith(new SetIndexerSpeed(indexer, Constants.ArmProfile.kIndexerDefaultOutput)));
    joystick.y().onFalse(armToZero);

    joystick.x().onTrue(armToAmp);
    joystick.x().whileTrue(setShooterSpeedForAmp.alongWith(new SetIndexerSpeed(indexer, Constants.ArmProfile.kIndexerDefaultOutput)));
    joystick.x().onFalse(armToZero);

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
