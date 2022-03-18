// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShotUtil;
import frc.robot.subsystems.CargoUtil;
import frc.robot.util.CargoState;
import frc.robot.util.ShotState;
import edu.wpi.first.wpilibj.Timer;

public class AutoShoot extends CommandBase {
  ShotUtil su;
  CargoUtil cu;
  Timer timer;
  boolean done;
  /** Creates a new AutoShoot. */
  public AutoShoot(ShotUtil su, CargoUtil cu) {
    this.su = su;
    this.cu = cu;
    addRequirements(this.su);
    addRequirements(this.cu);

    //Using a timer to make sure that balls are shot out
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    su.setState(ShotState.RUN_MOTOR_LOW_GOAL);
    cu.setState(CargoState.SPINUP);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (su.atRPM()){
      done = true;
    }
    su.operateShot();
    cu.OperateCargo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    su.setState(ShotState.STOP_MOTOR);
    su.operateShot();
    cu.setState(CargoState.IDLE);
    cu.OperateCargo();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
