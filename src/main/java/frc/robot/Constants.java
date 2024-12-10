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
    /* Id's and Port Assignments */
    public static final int pivotMotorA_ID = 15;
    public static final int pivotMotorB_ID = 16;
    public static final int indexerID = 17;
    public static final int shooterID_A = 18;
    public static final int shooterID_B = 19;

    /* Set Outputs */
    public static final double kShooterDefaultOutput = 0.90;
    public static final double kShooterAmpOutput = 0.40;
    public static final double kIndexerDefaultOutput = 1;

    /* Sensor references */
    public static final double noteStowedThreshold = 50;

    /* Current Limiting */
    public static final int kArmCurrentLimit = 20;

    /* Arm Threshold and Setpoints */
    public static final double kArmPosThreshold = 1;
    public static final double kArmInitialPos = 0;
    public static final double kArmSpeakerPos = 20;
    public static final double kArmAmpPos = 42;

    /* Arm PID Gains */
    public static final double kArmPIDKP = 0.1;
    public static final double kArmPIDKI = 0.0;
    public static final double kArmPIDKD = 0.0;

    /* Conversion Factors */
    public static final int neoEncoderCountsPerRev = 42;
    // public static final double kArmGearRatio = ((3.0 * 4.0 * 5.0) * 3.0) / 1;
    // public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    // public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    // public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;

    /* Soft Limits */
    public static final double kArmSoftLimitRvs = 0.0;
    public static final double kArmSoftLiimitFwd = 54000;
  }

}
