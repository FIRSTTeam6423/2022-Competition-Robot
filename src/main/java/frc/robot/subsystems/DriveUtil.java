// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveUtil extends SubsystemBase {
    private CANSparkMax leftPrimary, leftSecondary, rightPrimary, rightSecondary; 
    private RelativeEncoder leftPrimaryEncoder, leftSecondaryEncoder, rightPrimaryEncoder, rightSecondaryEncoder;
    private DifferentialDrive differentialDrive;

    // init motor controllers, set secondaries to follow and init DifferentialDrive controller
    public DriveUtil() {
        leftPrimary = new CANSparkMax(Constants.LEFT_PRIMARY, MotorType.kBrushless);
        leftSecondary = new CANSparkMax(Constants.LEFT_SECONDARY, MotorType.kBrushless);
        rightPrimary = new CANSparkMax(Constants.RIGHT_PRIMARY, MotorType.kBrushless);
        rightSecondary = new CANSparkMax(Constants.RIGHT_SECONDARY, MotorType.kBrushless);

        leftPrimaryEncoder = leftPrimary.getEncoder();
        leftSecondaryEncoder = leftSecondary.getEncoder();
        rightPrimaryEncoder = rightPrimary.getEncoder();
        rightSecondaryEncoder = rightSecondary.getEncoder();

        leftPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);

        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);

        // Invert secondaries (since they're on the opposite side of the robot)
        //leftSecondary.setInverted(true);
        //rightSecondary.setInverted(true);

        differentialDrive = new DifferentialDrive(leftPrimary, rightPrimary);
    }

    /**
     * Main function for driving the robot.
     * 
     * Gets driver inputs in the function rather then
     * having inputs passed in as parameters.
     * 
     * Drives in either arcade, tank or curvature drive mode
     * and uses the appropriate differential drive function to
     * move.
     */
    public void driveRobot() {
        double xboxLeftStickX = RobotContainer.getLeftXboxX();
        double xboxLeftStickY = RobotContainer.getLeftXboxY();
        double xboxRightStickY = RobotContainer.getRightXboxY();

        double xboxLeftTrigger = RobotContainer.getLeftXboxTrigger();
        double xboxRightTrigger = RobotContainer.getRightXboxTrigger();

        // arcade drive
        if (RobotContainer.driveType.getSelected().equals(RobotContainer.arcade)) {
            differentialDrive.arcadeDrive(-xboxLeftStickX, xboxRightStickY);

        // tank drive
        } else if (RobotContainer.driveType.getSelected().equals(RobotContainer.tank)) {
            differentialDrive.tankDrive(xboxLeftStickY, -xboxRightStickY);

        // curvature drive
        } else {
            // squaring the rotational input for additional precision;
            // although while differentialDrive should do it for us
            // drivers requested another one because turning felt imprecise
            // to them; according to them it helped
            double rotation = xboxLeftStickX;
            boolean isNegative = rotation < 0;
        
            rotation *= rotation;
            
            if (isNegative)
                rotation *= -1;

            rotation *= 0.75;

            differentialDrive.curvatureDrive(-rotation, xboxLeftTrigger - xboxRightTrigger, true);
        }
    }

    /**
     * Wrapper for the tankdrive funtion
     * Used for autos
     * 
     * @param leftSpeed  value passed into leftSpeed parameter of tankDrive function
     * @param rightSpeed value passed into rightSpeed parameter of tankDrive function
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }
    
    @Override
    public void periodic() {
        /** This is normally where we send important values to the SmartDashboard */

        SmartDashboard.putString("Drive Type   ::  ", RobotContainer.driveType.getSelected().toString());
        leftPrimaryEncoder.setPositionConversionFactor(4096);
        leftSecondaryEncoder.setPositionConversionFactor(4096);
        rightPrimaryEncoder.setPositionConversionFactor(4096);
        rightSecondaryEncoder.setPositionConversionFactor(4096);
        SmartDashboard.putNumber("Left Primary Encoder Ticks  ::  ", leftPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("Left Secondary Encoder Ticks  ::  ", leftSecondaryEncoder.getPosition());
        SmartDashboard.putNumber("Right Primary Encoder Ticks  ::  ", rightPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("Right Secondary Encoder Ticks  ::  ", rightSecondaryEncoder.getPosition());

    }
}
