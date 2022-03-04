// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoUtil;
import frc.robot.util.CargoState;
import edu.wpi.first.wpilibj.Timer;

public class AutoShoot extends CommandBase {
  CargoUtil cu;
  Timer timer;
  boolean done;
  /** Creates a new AutoShoot. */
  public AutoShoot(CargoUtil cu) {
    this.cu = cu;
    addRequirements(this.cu);

    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    cu.setState(CargoState.SPINUP);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 5){
      done = true;
    }
    cu.OperateCargo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cu.setState(CargoState.IDLE);
    cu.OperateCargo();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
