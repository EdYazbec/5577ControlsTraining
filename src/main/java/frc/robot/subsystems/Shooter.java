// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmProfile;

public class Shooter extends SubsystemBase {
    /** Shooter Motors */
    private WPI_TalonSRX shooterA = new WPI_TalonSRX(ArmProfile.shooterID_A); //18
    private WPI_TalonSRX shooterB = new WPI_TalonSRX(ArmProfile.shooterID_B); //19

    /** Initaliaztion Box */
    public Shooter() {
        /** Factory Resets */
        shooterA.configFactoryDefault();
        shooterB.configFactoryDefault();

        /** Inversion Factors */
        shooterA.setInverted(false);
        shooterB.setInverted(false);

        /** Current Limits */
        shooterA.configPeakCurrentLimit(30);
        shooterB.configPeakCurrentLimit(30);
    }

    /**
     * Runs both shooter motors at commanded percentage
     * 
     * @param outputValue commanded output value, -1 to 1
     */
    public void setShooterOutput(double outputValue) {
        shooterA.set(outputValue);
        shooterB.set(outputValue);
    } 

    @Override
    /** This method will be called once per scheduler run */
    public void periodic() {
    }
}
