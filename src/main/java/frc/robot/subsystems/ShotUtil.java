// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants;
import frc.robot.util.ShotState;

public class ShotUtil extends PIDSubsystem {
  /** Creates a new ShotUtil. */
  private CANSparkMax shooter;
  private RelativeEncoder shooterEncoder;
  private ShotState state = ShotState.STOP_MOTOR;

  public ShotUtil() {
    super(new PIDController(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D));

    shooter = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);
    shooter.setIdleMode(IdleMode.kCoast);

    shooterEncoder = shooter.getEncoder();

    setSetpoint(0);
    getController().setTolerance(Constants.SHOOTER_POSITION_TOLERANCE, Constants.SHOOTER_VELOCITY_TOLERANCE);
  }

  public void setState(ShotState newState){
    state = newState;
    System.out.println(newState.toString());
    System.out.println(state.toString());
  }

  public ShotState getState(){
    return state;
  }

  public void operateShot(){
    if (state == ShotState.RUN_MOTOR){
      if(!isEnabled()){
        enable();
      }
      setSetpoint(Constants.SHOOTER_RPM);
    } else {
      disable();
      setSetpoint(0);
    }
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Ensure the output value from the PID Controller is between -1 to 1.
    // You can't set a motor higher than 1.0/lower than -1.0.
    output = MathUtil.clamp(output, -1.0, 1.0);

    // Set your motor speed to the PID Controller's output value.
    shooter.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here

    // Return your encoder's velocity measurement.
    // On Spark Max motor controllers, this is the motor's RPM.
    return shooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putBoolean("At RPM", getController().atSetpoint());

    SmartDashboard.putString("Shoot Mode", getState().toString());
  }
}
