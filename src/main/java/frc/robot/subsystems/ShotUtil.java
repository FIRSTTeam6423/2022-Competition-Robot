// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;
import frc.robot.util.ShotState;

public class ShotUtil extends SubsystemBase {
  /** Creates a new ShotUtil. */
  private CANSparkMax shooter;
  private RelativeEncoder shooterEncoder;
  private SparkMaxPIDController shooterPIDController; 
  private ShotState state;

  public ShotUtil() {
    shooter = new CANSparkMax(Constants.SHOOTER, MotorType.kBrushless);

    shooter.setInverted(true);

    shooterEncoder = shooter.getEncoder();
    shooterPIDController = shooter.getPIDController();

    shooterPIDController.setP(Constants.SHOOTER_P);
    shooterPIDController.setI(Constants.SHOOTER_I);
    shooterPIDController.setD(Constants.SHOOTER_D);
    shooterPIDController.setFF(Constants.SHOOTER_F);

  }

  public void operateShooter(){
    shooterPIDController.setReference(Constants.SHOOTER_RPM, CANSparkMax.ControlType.kVelocity);
  }

  public double getShooterRPM(){
    return shooterEncoder.getVelocity();
  }

  public void stopShooter(){
    shooterPIDController.setReference(0.0, CANSparkMax.ControlType.kVelocity);
  }

  public ShotState returnState(){
    return state;
  }

  public void setState(ShotState newState){
    state = newState;
  }

  public void operateShot(){
    switch(state){
      case SHOOT:
        operateShooter();
        break;
      case IDLE:
        stopShooter();
        break;
    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
