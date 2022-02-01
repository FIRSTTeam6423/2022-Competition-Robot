// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;

public class SensorUtil extends SubsystemBase{
    //Color detector
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    Color detectedColor = m_colorSensor.getColor();
    int proximity = m_colorSensor.getProximity();

    //Ball detector
    private DigitalInput limitSwitch;

    public SensorUtil(){
        limitSwitch = new DigitalInput(1);
    }

    /**
     * Constantly check the rgb values read by the color sensor
     * If they match the values of red cargo, display "RED" on the dashboard
     * If they match the values of blue cargo, display "BLUE" on the dashboard
     * If the rgb values match neither types of cargo, display "NO COLOR DETECTED" on the dashboard
     */
    public void detectBallColor(){
        if (detectedColor.red > 0.55 && detectedColor.blue < 0.1){
            SmartDashboard.putString("color detected", "RED");
        } else if (detectedColor.blue > 0.3){
            SmartDashboard.putString("color detected", "BLUE");
        } else {
            SmartDashboard.putString("color detected", "NO COLOR DETECTED");
        }
    }

    public void detectBall(){
        if (limitSwitch.get()){
            SmartDashboard.putString("ball detected", "BALL DETECTED");
        } else {
            SmartDashboard.putString("ball detected", "NO BALLs DETECTED");
        }
    }

    public void detectGyro(){
        SmartDashboard.putNumber("Rate", gyro.getRate());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putNumber("Heading", gyro.getCompassHeading());
        SmartDashboard.putNumber("LinearWorldAccelX", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("LinearWorldAccelY", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("LinearWorldAccelZ", gyro.getWorldLinearAccelZ());
    }
}
