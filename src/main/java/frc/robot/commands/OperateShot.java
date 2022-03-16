// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShotUtil;
import frc.robot.util.ShotState;

public class OperateShot extends CommandBase {
  /** Creates a new OperateShot. */
  private final ShotUtil su;

  public OperateShot(ShotUtil su) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.su = su;
    addRequirements(su);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // su.setState(ShotState.STOP_MOTOR);
    // su.operateShot();
    System.out.println(su.getState().toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    su.operateShot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // su.setState(ShotState.STOP_MOTOR);
    // su.operateShot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
