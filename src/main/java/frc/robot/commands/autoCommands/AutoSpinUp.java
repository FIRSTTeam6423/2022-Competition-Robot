// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enums.CargoState;
import frc.robot.subsystems.CargoUtil;

public class AutoSpinUp extends CommandBase {
  /** Creates a new AutoSpinUp. */
  CargoUtil cu;
  public AutoSpinUp(CargoUtil cu) {
    this.cu = cu;
    addRequirements(this.cu);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cu.setState(CargoState.SPINUP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
