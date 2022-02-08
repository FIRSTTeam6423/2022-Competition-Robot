// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveUtil extends SubsystemBase {
    // Motor controllers
    private CANSparkMax leftSecondary, rightSecondary, leftPrimary, rightPrimary;

    // Drive controller
    private DifferentialDrive differentialDrive;

    private double damp;

    public DriveUtil() {
      leftPrimary = new CANSparkMax(Constants.LEFT_PRIMARY, MotorType.kBrushless);
      leftSecondary = new CANSparkMax(Constants.LEFT_SECONDARY, MotorType.kBrushless);
      rightPrimary = new CANSparkMax(Constants.RIGHT_PRIMARY, MotorType.kBrushless);
      rightSecondary = new CANSparkMax(Constants.RIGHT_SECONDARY, MotorType.kBrushless);

        // Set secondaries to follow primaries
        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);

        damp = 0.0;

        // Invert secondaries (since they're on the opposite side of the robot)
        //leftSecondary.setInverted(true);
        //rightSecondary.setInverted(true);

        // Initialize DifferentialDrive controller
        differentialDrive = new DifferentialDrive(leftPrimary, rightPrimary);
    }

    /**
     * Drive the robot based on the driveMode class parameter.
     * If in TANK mode, use leftX and rightX values.
     * If in ARCADE mode, use rightX and rightY values.
     * 
     * The DifferentialDrive class will square inputs for us.
     * Squaring inputs results in less sensitive inputs.
     * 
     * @param leftX the left controller's X (forward-backward) value
     * @param leftY the left controller's Y (left-right) value
     * @param rightX the right controller's X (forward-backward) value
     * @param rightY the right controller's Y (left-right) value
     */
    public void driveRobot() {
        if (RobotContainer.driveType.getSelected().equals(RobotContainer.arcade)) {
        // If we're in ARCADE mode, use arcadeDrive
        differentialDrive.arcadeDrive(RobotContainer.getDriverLeftXboxX(), -RobotContainer.getDriverRightXboxY());
        } else if (RobotContainer.driveType.getSelected().equals(RobotContainer.tank)) {
        // If we're in TANK mode, use tankDrive
        differentialDrive.tankDrive(-RobotContainer.getDriverLeftXboxY(), RobotContainer.getDriverRightXboxY());
        } else {
        // If we are in CURVATURE mode, use the curvature mode
        double rotation = RobotContainer.getDriverLeftXboxX();
        boolean isNegative = rotation < 0;
        
        rotation *= rotation;
        if (isNegative){
          rotation *= -1;
        }
        rotation *= 0.75;

        differentialDrive.curvatureDrive(rotation, -RobotContainer.getDriverLeftXboxTrigger() + RobotContainer.getDriverRightXboxTrigger(), true);}
      }
    
    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Drive Type   ::  ", RobotContainer.driveType.getSelected().toString());
    }
}

