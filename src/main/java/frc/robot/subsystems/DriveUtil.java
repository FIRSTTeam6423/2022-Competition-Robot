// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveUtil extends SubsystemBase {
    private CANSparkMax leftPrimary, leftSecondary, rightPrimary, rightSecondary;
    private RelativeEncoder leftPrimaryEncoder, leftSecondaryEncoder, rightPrimaryEncoder, rightSecondaryEncoder;
    public double setpoint, rotation;

    // Drive controller
    private DifferentialDrive differentialDrive;

    // private SparkMaxPIDController leftDriverPIDController,
    // rightDriverPIDController;

    public DriveUtil() {
        leftPrimary = new CANSparkMax(Constants.LEFT_PRIMARY, MotorType.kBrushless);
        leftSecondary = new CANSparkMax(Constants.LEFT_SECONDARY, MotorType.kBrushless);
        rightPrimary = new CANSparkMax(Constants.RIGHT_PRIMARY, MotorType.kBrushless);
        rightSecondary = new CANSparkMax(Constants.RIGHT_SECONDARY, MotorType.kBrushless);

        // leftDriverPIDController = leftPrimary.getPIDController();
        // rightDriverPIDController = rightPrimary.getPIDController();

        leftPrimaryEncoder = leftPrimary.getEncoder();
        leftSecondaryEncoder = leftSecondary.getEncoder();
        rightPrimaryEncoder = rightPrimary.getEncoder();
        rightSecondaryEncoder = rightSecondary.getEncoder();

        leftPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);

        /**
         * Conversion factor was 4096 before, from this CD forum thread setting
         * conversion factor to 42 gets this to effectively report ticks rather than
         * rotation units. This is untested but seems like the right approach.
         * https://www.chiefdelphi.com/t/neo-motor-encoder-ticks-per-roataion/347126/2
         */
        leftPrimaryEncoder.setPositionConversionFactor(42);
        leftSecondaryEncoder.setPositionConversionFactor(42);
        rightPrimaryEncoder.setPositionConversionFactor(42);
        rightSecondaryEncoder.setPositionConversionFactor(42);

        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);

        rightPrimary.setInverted(true);

        /**
         * Having PID on the drive base isn't necessarily needed, especially for normal
         * match play. PID could help move the robot a precise distance in a short
         * amount of time. The same can be achieved without PID by instead driving the
         * robot slower and using encoder ticks. It may be a good idea to get distance
         * driving functional without PID before introducing that - you may find not
         * having PID works well enough!
         */

        // leftDriverPIDController.setP(Constants.DRIVER_P);
        // leftDriverPIDController.setI(Constants.DRIVER_I);
        // leftDriverPIDController.setD(Constants.DRIVER_D);
        // leftDriverPIDController.setFF(Constants.DRIVER_F);

        // rightDriverPIDController.setP(Constants.DRIVER_P);
        // rightDriverPIDController.setI(Constants.DRIVER_I);
        // rightDriverPIDController.setD(Constants.DRIVER_D);
        // rightDriverPIDController.setFF(Constants.DRIVER_F);

        setpoint = 0;

        // Invert secondaries (since they're on the opposite side of the robot)
        // leftSecondary.setInverted(true);
        // rightSecondary.setInverted(true);

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
        /**
         * Creating multiple double objects in a periodic method (driveRobot is called
         * 100 times per second) can overwork the JVM garbage collection process.
         * Since most of these are only accessed once, call the get method directly
         * rather than creating doubles and referencing them.
         */
        // double xboxLeftStickX = RobotContainer.getDriverLeftXboxX();
        // double xboxLeftStickY = RobotContainer.getDriverLeftXboxY();
        // double xboxRightStickY = RobotContainer.getDriverRightXboxY();

        // double xboxLeftTrigger = RobotContainer.getDriverLeftXboxTrigger();
        // double xboxRightTrigger = RobotContainer.getDriverRightXboxTrigger();

        // arcade drive
        if (RobotContainer.driveType.getSelected().equals(RobotContainer.arcade)) {
            differentialDrive.arcadeDrive(-RobotContainer.getDriverLeftXboxX(), RobotContainer.getDriverRightXboxY());

            // tank drive
        } else if (RobotContainer.driveType.getSelected().equals(RobotContainer.tank)) {
            differentialDrive.tankDrive(RobotContainer.getDriverLeftXboxY(), -RobotContainer.getDriverRightXboxY());

            // curvature drive
        } else {
            // squaring the rotational input for additional precision;
            // although while differentialDrive should do it for us
            // drivers requested another one because turning felt imprecise
            // to them; according to them it helped

            /**
             * Suggested to move `rotation` to a class variable rather than redefining it
             * every method call. Same reason as mentioned above for xbox values - it can
             * overwork the JVM garbage collector.
             */
            rotation = RobotContainer.getDriverLeftXboxX();
            boolean isNegative = rotation < 0;

            rotation *= rotation;

            if (isNegative)
                rotation *= -1;

            rotation *= 0.75;

            /**
             * Moved `rotation` to the second parameter slot. curvatureDrive has a method
             * signature of `double xSpeed, double zRotation, boolean allowTurnInPlace`.
             * Unless this `rotation` value isn't a rotation value, it likely belongs in the
             * second parameter slot.
             */
            differentialDrive.curvatureDrive(
                    RobotContainer.getDriverLeftXboxTrigger() - RobotContainer.getDriverRightXboxTrigger(), -rotation,
                    true);
        }
    }

    /**
     * Wrapper for the tankdrive funtion
     * Used for autos
     * 
     * @param leftSpeed  value passed into leftSpeed parameter of tankDrive function
     * @param rightSpeed value passed into rightSpeed parameter of tankDrive
     *                   function
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * It's very useful to reset encoders while doing distance calculations to
     * ensure you're getting accurate measurements. It also saves you from having to
     * calculate deltas. Also, encoders can "walk" over time, meaning their readings
     * become less accurate with the real measurement. Resetting helps avoid all
     * those issues.
     */
    public void resetEncoders() {
        leftPrimaryEncoder.setPosition(0);
        leftSecondaryEncoder.setPosition(0);
        rightPrimaryEncoder.setPosition(0);
        rightSecondaryEncoder.setPosition(0);
    }

    public void operateDistance(double distance) {
        // leftDriverPIDController.setReference(distance,
        // CANSparkMax.ControlType.kPosition);
        // rightDriverPIDController.setReference(distance,
        // CANSparkMax.ControlType.kPosition);
        // setpoint = distance;
    }

    public void stopDistance() {
        // leftDriverPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
        // rightDriverPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Not sure of the benefit of this method. A command being considered done
     * shouldn't be dependent on the speed a motor is spinning at. Think of a motor
     * like an "output" rather than an "input" in a computer. You can't watch a
     * YouTube video with only a keyboard. Similarly, you shouldn't get "readings"
     * from a motor. If you need an "input" regarding motors, that's what encoders
     * and gyros are for!
     */
    public boolean getMoving() {
        return leftPrimary.get() > 0.1 && rightSecondary.get() > 0.1;
    }

    /**
     * Averaging encoder readings is good on one hand because it accounts for any
     * inconsistencies in the hardware. However, this assumes the encoders are in
     * sync with each other. If the left is 10 ticks ahead of the right (powered on
     * sooner, left drive train skipped when going over a bump), this measurement is
     * much less realistic. Calling `resetEncoders()` would help with that. However,
     * it's much more advisable to instead read from one encoder.
     * 
     * Having two encoders on a robot like this is more for having a backup than
     * actively using both. If you ever need to keep the robot driving straight, use
     * a gyro instead.
     */
    public double getPosition() {
        return (leftPrimaryEncoder.getPosition() + rightPrimaryEncoder.getPosition()) / 2;
    }

    public double getLeftEncoder() {
        return leftPrimaryEncoder.getPosition();
    }

    public double getRightEncoder() {
        return rightPrimaryEncoder.getPosition();
    }

    @Override
    public void periodic() {
        /** This is normally where we send important values to the SmartDashboard */
        SmartDashboard.putString("Drive Type   ::  ", RobotContainer.driveType.getSelected().toString());

        /**
         * This shouldn't go in a periodic method! This method is called 100 times per
         * second - there is no need to set the conversion factor periodically.
         */
        // leftPrimaryEncoder.setPositionConversionFactor(4096);
        // leftSecondaryEncoder.setPositionConversionFactor(4096);
        // rightPrimaryEncoder.setPositionConversionFactor(4096);
        // rightSecondaryEncoder.setPositionConversionFactor(4096);
        SmartDashboard.putNumber("Left Primary Encoder Ticks  ::  ", leftPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("Left Secondary Encoder Ticks  ::  ", leftSecondaryEncoder.getPosition());
        SmartDashboard.putNumber("Right Primary Encoder Ticks  ::  ", rightPrimaryEncoder.getPosition());
        SmartDashboard.putNumber("Right Secondary Encoder Ticks  ::  ", rightSecondaryEncoder.getPosition());
        SmartDashboard.putNumber("Distance Setpoint ::  ", setpoint);

    }
}
