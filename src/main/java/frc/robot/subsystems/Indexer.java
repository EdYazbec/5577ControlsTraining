// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmProfile;


public class Indexer extends SubsystemBase {
    // in here, we are declaring what things exist on the indexer. 
    // in our case, it has a motor, and  a time of flight (distance) sensor

    /** Indexer Motor */
    private WPI_TalonSRX motor = new WPI_TalonSRX(ArmProfile.indexerID);

    /** Indexer Sensors */
    private TimeOfFlight sensor = new TimeOfFlight(49);
        
    /** Creates a new Indexer. */
    public Indexer() {
        motor.configFactoryDefault();
        motor.setInverted(true);
        motor.configPeakCurrentLimit(20);
    }

    /**
     * Runs both shooter motors at commanded percentage
     * 
     * @param outputValue commanded output value, -1 to 1
     */
    public void setOutput(double outputValue) {
        motor.set(outputValue);
    } 

    /**
     * Checks if note is inside the middle of the indexer
     * 
     * @return true if the note breaks beam sensor, otherwise false
     */
    public boolean noteStowed() {
        if (sensor.getRange() <= Constants.ArmProfile.noteStowedThreshold)
            return true;
        else
            return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
