// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;

import com.kauailabs.navx.frc.AHRS;

public class DriveUtil extends SubsystemBase {
    private CANSparkMax leftPrimary, leftSecondary, rightPrimary, rightSecondary; 
    private RelativeEncoder leftPrimaryEncoder, leftSecondaryEncoder, rightPrimaryEncoder, rightSecondaryEncoder;

    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private DifferentialDrive differentialDrive;
    private SparkMaxPIDController leftDriverPIDController, rightDriverPIDController; 

    public double setpoint;

    public DriveUtil() {       
        leftPrimary = new CANSparkMax(Constants.LEFT_PRIMARY, MotorType.kBrushless);
        leftSecondary = new CANSparkMax(Constants.LEFT_SECONDARY, MotorType.kBrushless);
        rightPrimary = new CANSparkMax(Constants.RIGHT_PRIMARY, MotorType.kBrushless);
        rightSecondary = new CANSparkMax(Constants.RIGHT_SECONDARY, MotorType.kBrushless);

        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);

        rightPrimary.setInverted(false);

        leftPrimaryEncoder = leftPrimary.getEncoder();
        leftSecondaryEncoder = leftSecondary.getEncoder();
        rightPrimaryEncoder = rightPrimary.getEncoder();
        rightSecondaryEncoder = rightSecondary.getEncoder();

        leftPrimaryEncoder.setPositionConversionFactor(4096);
        leftSecondaryEncoder.setPositionConversionFactor(4096);
        rightPrimaryEncoder.setPositionConversionFactor(4096);
        rightSecondaryEncoder.setPositionConversionFactor(4096);

        // PID init and config
        // PIDs aren't currently used but were leaving them in case we decide to switch
        leftDriverPIDController = leftPrimary.getPIDController();
        rightDriverPIDController = rightPrimary.getPIDController();

        leftDriverPIDController.setP(Constants.DRIVER_P);
        leftDriverPIDController.setI(Constants.DRIVER_I);
        leftDriverPIDController.setD(Constants.DRIVER_D);
        leftDriverPIDController.setFF(Constants.DRIVER_F);

        rightDriverPIDController.setP(Constants.DRIVER_P);
        rightDriverPIDController.setI(Constants.DRIVER_I);
        rightDriverPIDController.setD(Constants.DRIVER_D);
        rightDriverPIDController.setFF(Constants.DRIVER_F);

        setpoint = 0;


        differentialDrive = new DifferentialDrive(leftPrimary, rightPrimary);

        CameraServer.startAutomaticCapture();
        gyro.calibrate();
        gyro.reset();
    }

    
    public void driveRobot() {
        if (RobotContainer.driveType.getSelected().equals(RobotContainer.arcade)) { 
            differentialDrive.arcadeDrive(RobotContainer.getDriverRightXboxX(), -RobotContainer.getDriverRightXboxY());
        } else if (RobotContainer.driveType.getSelected().equals(RobotContainer.tank)) { 
            differentialDrive.tankDrive(-RobotContainer.getDriverLeftXboxY(), RobotContainer.getDriverRightXboxY());
        } else {
            // We square the rotational input as the drivers requested additional precision
            double rotation = RobotContainer.getDriverLeftXboxX();
            boolean isNegative = rotation < 0;

            rotation *= rotation;
            if (isNegative)
                rotation *= -1;

            rotation *= 0.75;

            differentialDrive.curvatureDrive(rotation, -RobotContainer.getDriverLeftXboxTrigger() + RobotContainer.getDriverRightXboxTrigger(), true);
        }
    }

    // Wrapper for tank drive, used by auto commands
    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void operateDistance(double distance){
        leftDriverPIDController.setReference(distance, CANSparkMax.ControlType.kPosition);
        rightDriverPIDController.setReference(distance, CANSparkMax.ControlType.kPosition);
        setpoint = distance;
    }

    public void stopDistance(){
        leftDriverPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
        rightDriverPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    public boolean getMoving(){
        return leftPrimary.get() > 0.1 && rightSecondary.get() > 0.1;
    }

    public double getPosition(){
        return (leftPrimaryEncoder.getPosition() + rightPrimaryEncoder.getPosition()) / 2;
    }

    public double getLeftPosition() {
        return leftPrimaryEncoder.getPosition();
    }   

    public double getRightPosition() {
        return rightPrimaryEncoder.getPosition();
    }   

    public void resetEncoder(){
        leftPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);
    }

    public double getGyroYaw(){
        return gyro.getYaw();
    }

    public void resetGyro(){
        gyro.reset();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putString("Drive Type   ::  ", RobotContainer.driveType.getSelected().toString());
        SmartDashboard.putNumber("Yaw", getGyroYaw());
        SmartDashboard.putNumber("Encoder", getLeftPosition());
    }
}

