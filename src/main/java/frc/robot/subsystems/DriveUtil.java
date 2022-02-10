// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import com.revrobotics.SparkMaxPIDController;

public class DriveUtil extends SubsystemBase {
    private CANSparkMax leftPrimary, leftSecondary, rightPrimary, rightSecondary; 
    private RelativeEncoder leftPrimaryEncoder, leftSecondaryEncoder, rightPrimaryEncoder, rightSecondaryEncoder;
    public double setpoint;

    // Drive controller
    private DifferentialDrive differentialDrive;

    private SparkMaxPIDController leftDriverPIDController, rightDriverPIDController; 

    private double damp;

    public DriveUtil() {
        leftPrimary = new CANSparkMax(Constants.LEFT_PRIMARY, MotorType.kBrushless);
        leftSecondary = new CANSparkMax(Constants.LEFT_SECONDARY, MotorType.kBrushless);
        rightPrimary = new CANSparkMax(Constants.RIGHT_PRIMARY, MotorType.kBrushless);
        rightSecondary = new CANSparkMax(Constants.RIGHT_SECONDARY, MotorType.kBrushless);

        leftPrimaryEncoder = leftPrimary.getEncoder();
        leftSecondaryEncoder = leftSecondary.getEncoder();
        rightPrimaryEncoder = rightPrimary.getEncoder();
        rightSecondaryEncoder = rightSecondary.getEncoder();

        leftDriverPIDController = leftPrimary.getPIDController();
        rightDriverPIDController = rightPrimary.getPIDController();

        leftPrimaryEncoder.setPositionConversionFactor(4096);
        leftSecondaryEncoder.setPositionConversionFactor(4096);
        rightPrimaryEncoder.setPositionConversionFactor(4096);
        rightSecondaryEncoder.setPositionConversionFactor(4096);

        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);

        rightPrimary.setInverted(true);

        leftDriverPIDController.setP(Constants.DRIVER_P);
        leftDriverPIDController.setI(Constants.DRIVER_I);
        leftDriverPIDController.setD(Constants.DRIVER_D);
        leftDriverPIDController.setFF(Constants.DRIVER_F);

        rightDriverPIDController.setP(Constants.DRIVER_P);
        rightDriverPIDController.setI(Constants.DRIVER_I);
        rightDriverPIDController.setD(Constants.DRIVER_D);
        rightDriverPIDController.setFF(Constants.DRIVER_F);

        setpoint = 0;

        // Set secondaries to follow primaries
        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);

        damp = 0.0;

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
        double xboxLeftStickX = RobotContainer.getDriverLeftXboxX();
        double xboxLeftStickY = RobotContainer.getDriverLeftXboxY();
        double xboxRightStickY = RobotContainer.getDriverRightXboxY();

        double xboxLeftTrigger = RobotContainer.getDriverLeftXboxTrigger();
        double xboxRightTrigger = RobotContainer.getDriverRightXboxTrigger();

        // arcade drive
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
        differentialDrive.tankDrive(leftSpeed, -rightSpeed);
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
        double sensorPosition = (leftPrimaryEncoder.getPosition() + rightPrimaryEncoder.getPosition())/2;

        return sensorPosition;
    }

    public double getleftPosition() {
        double leftSensorPosition = leftPrimaryEncoder.getPosition();

        return leftSensorPosition;
    }   

    public double getrightPosition() {
        double rightSensorPosition = rightPrimaryEncoder.getPosition();

        return rightSensorPosition;
    }   

    public void resetEndconder(){
        leftPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Drive Type   ::  ", RobotContainer.driveType.getSelected().toString());
        SmartDashboard.putNumber("Left Primary Encoder Ticks  ::  ", leftPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("Left Secondary Encoder Ticks  ::  ", leftSecondaryEncoder.getPosition());
        SmartDashboard.putNumber("Right Primary Encoder Ticks  ::  ", rightPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("Right Secondary Encoder Ticks  ::  ", rightSecondaryEncoder.getPosition());
        SmartDashboard.putNumber("Distance Setpoint ::  ", setpoint);
    }
}

