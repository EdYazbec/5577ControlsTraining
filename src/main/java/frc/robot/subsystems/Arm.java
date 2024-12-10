// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmProfile;

public class Arm extends SubsystemBase {
    /** Pivot Motors */
    private CANSparkMax motorA = new CANSparkMax(ArmProfile.pivotMotorA_ID, MotorType.kBrushless);
    private CANSparkMax motorB = new CANSparkMax(ArmProfile.pivotMotorB_ID, MotorType.kBrushless);

    /** Pivot PID Controllers */
    private SparkPIDController pivotControllerA = motorA.getPIDController();
    private SparkPIDController pivotControllerB = motorB.getPIDController();

    /** Pivot Motor Controllers */
    private RelativeEncoder pivotEncoderA = motorA.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);
    private RelativeEncoder pivotEncoderB = motorB.getEncoder(SparkRelativeEncoder.Type.kHallSensor, ArmProfile.neoEncoderCountsPerRev);

    private double setpoint = 0;

    /** Creates a new Arm. */
    public Arm() {
        /** Factory Resets */
        motorA.restoreFactoryDefaults();
        motorB.restoreFactoryDefaults();

        /** Inversion Factors */
        motorA.setInverted(true);
        motorB.setInverted(false);
        
        /** Current Limits */
        motorA.setSmartCurrentLimit(ArmProfile.kArmCurrentLimit);
        motorB.setSmartCurrentLimit(ArmProfile.kArmCurrentLimit);

        /** Arm Neutral Modes */
        motorA.setIdleMode(IdleMode.kCoast);
        motorB.setIdleMode(IdleMode.kCoast);

        /* Conversion Factors */
        pivotEncoderA.setPosition(ArmProfile.kArmInitialPos);
        pivotEncoderB.setPosition(ArmProfile.kArmInitialPos);

        // Set the P value of the PID controller on slot 0 to 0.25
        pivotControllerA.setP(Constants.ArmProfile.kArmPIDKP);
        pivotControllerA.setI(Constants.ArmProfile.kArmPIDKI);
        pivotControllerB.setP(Constants.ArmProfile.kArmPIDKP);
        pivotControllerB.setI(Constants.ArmProfile.kArmPIDKI);
        pivotControllerB.setD(Constants.ArmProfile.kArmPIDKD);

        /** Flash Arm Controllers Configs */
        System.out.println(motorA.burnFlash());
        System.out.println(motorB.burnFlash());
    }

    /* Basic pivot functions */
    /** 
     * Sets the arm PID controller setpoint and moves the arm to that setpoint
     * 
     * @param armSetpoint the setpoint for the arm
     */
    public void setArmPos(double setpoint) {
        this.setpoint = setpoint;
        pivotControllerA.setReference(setpoint, ControlType.kPosition);
        pivotControllerB.setReference(setpoint, ControlType.kPosition);
        // motorB.follow(motorA);
    }

    public void setArmSpeed(double speed)
    {
        motorA.set(speed);
        motorB.set(speed);
        // motorB.follow(motorA);
    }

    /**
     *  Checks if the arm is near the setpoint
     * 
     * @return true if the arm is at the set point, otherwise false
     */
    public boolean isAtSetpoint() {
        double lowerLimit = setpoint - ArmProfile.kArmPosThreshold;
        double upperLimit = setpoint + ArmProfile.kArmPosThreshold;
        if ((lowerLimit <= pivotEncoderA.getPosition()) && (pivotEncoderA.getPosition() <= upperLimit))
            return true;
        else
            return false;
    }

    /**
     *  Checks if the arm is below the setpoint
     * 
     * @return true if the arm is below the set point, otherwise false
     */
    public boolean isBelowSetpoint() {
        if (pivotEncoderA.getPosition() <= setpoint)
            return true;
        else
            return false;
    }

    /**
     *  Checks if the arm is above the setpoint
     * 
     * @return true if the arm is above the set point, otherwise false
     */
    public boolean isAboveSetpoint() {
        if (setpoint <= pivotEncoderA.getPosition()) 
            return true;
        else
            return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("ArmPoseA", pivotEncoderA.getPosition());
        SmartDashboard.putNumber("ArmPoseB", pivotEncoderB.getPosition());
        SmartDashboard.putNumber("ArmSetpoint", setpoint);
        SmartDashboard.putBoolean("ArmAboveSetpoint", isAboveSetpoint());
        SmartDashboard.putBoolean("ArmBelowSetpoint", isBelowSetpoint());
        SmartDashboard.putBoolean("ArmAtSetpoint", isAtSetpoint());
    }
}
