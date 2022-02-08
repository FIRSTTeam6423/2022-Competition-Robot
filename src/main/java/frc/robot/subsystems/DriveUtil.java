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
import com.revrobotics.SparkMaxPIDController;

public class DriveUtil extends SubsystemBase {
    private CANSparkMax leftPrimary, leftSecondary, rightPrimary, rightSecondary; 
    private RelativeEncoder leftPrimaryEncoder, leftSecondaryEncoder, rightPrimaryEncoder, rightSecondaryEncoder;
    public double setpoint;

    // Drive controller
    private DifferentialDrive differentialDrive;

    private SparkMaxPIDController leftDriverPIDController, rightDriverPIDController; 

    public DriveUtil() {
        leftPrimary = new CANSparkMax(Constants.LEFT_PRIMARY, MotorType.kBrushless);
        leftSecondary = new CANSparkMax(Constants.LEFT_SECONDARY, MotorType.kBrushless);
        rightPrimary = new CANSparkMax(Constants.RIGHT_PRIMARY, MotorType.kBrushless);
        rightSecondary = new CANSparkMax(Constants.RIGHT_SECONDARY, MotorType.kBrushless);

        leftDriverPIDController = leftPrimary.getPIDController();
        rightDriverPIDController = rightPrimary.getPIDController();

        leftPrimaryEncoder = leftPrimary.getEncoder();
        leftSecondaryEncoder = leftSecondary.getEncoder();
        rightPrimaryEncoder = rightPrimary.getEncoder();
        rightSecondaryEncoder = rightSecondary.getEncoder();

        leftPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);

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
        double xboxLeftStickX = RobotContainer.getDriverLeftXboxX();
        double xboxLeftStickY = RobotContainer.getDriverLeftXboxY();
        double xboxRightStickY = RobotContainer.getDriverRightXboxY();

        double xboxLeftTrigger = RobotContainer.getDriverLeftXboxTrigger();
        double xboxRightTrigger = RobotContainer.getDriverRightXboxTrigger();

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
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Drive Type   ::  ", RobotContainer.driveType.getSelected().toString());
        SmartDashboard.putNumber("Left Primary Encoder Ticks  ::  ", leftPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("Left Secondary Encoder Ticks  ::  ", leftSecondaryEncoder.getPosition());
        SmartDashboard.putNumber("Right Primary Encoder Ticks  ::  ", rightPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("Right Secondary Encoder Ticks  ::  ", rightSecondaryEncoder.getPosition());
        SmartDashboard.putNumber("Distance Setpoint ::  ", setpoint);

    }
}
