// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static class ControllerProfile {
    /* Controller Port Assingments */
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final int kTechnitionControllerPort = 2;

    /* Joystick Deadband */
    public static final double stickDeadband = 0.18;
  }

  public static class ArmProfile {
    /* Id's and Port Assingments */
    public static final int shooterID_A = 18;
    public static final int shooterID_B = 19;

    /* Set Outputs */
    public static final double kShooterDefaultOutput = 0.90;
    public static final double kShooterAmpOutput = 0.40;

  }

}
