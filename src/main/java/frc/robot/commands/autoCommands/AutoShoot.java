// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShotUtil;
import frc.robot.Constants;
import frc.robot.subsystems.CargoUtil;
import edu.wpi.first.wpilibj.Timer;

public class AutoShoot extends CommandBase {
  ShotUtil su;
  CargoUtil cu;
  Timer timer;
  boolean done;
  
  /** Creates a new AutoShoot. */
  public AutoShoot(ShotUtil su) {
    this.su = su;
    addRequirements(this.su);

    //Using a timer to make sure that balls are shot out
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!su.isEnabled()){
      su.enable();
    }
    su.setSetpoint(Constants.LOW_GOAL_SHOOTER_RPM);
    if(su.atRPM()){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
