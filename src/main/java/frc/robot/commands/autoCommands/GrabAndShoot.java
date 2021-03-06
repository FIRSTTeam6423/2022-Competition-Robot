// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveUtil;
import frc.robot.Constants;
import frc.robot.subsystems.CargoUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabAndShoot extends SequentialCommandGroup {
  /** Creates a new GrabNShoot. */
  public GrabAndShoot(DriveUtil du, CargoUtil cu) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new DriveForDistanceNoPID(du, 60, true),
        new AutoIntake(cu)
      ),
      new TurnForAngle(du, 170),
      new DriveForDistanceNoPID(du, 80, false),
      new DriveForTime(du, 3.5, Constants.AUTO_DRIVE_SPEED),
      new DriveForDistanceNoPID(du, -6, true),
      new AutoShoot(cu)
    );
  }
}
