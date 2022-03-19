// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveUtil;
import frc.robot.util.ShotState;
import frc.robot.subsystems.ShotUtil;
import frc.robot.Constants;
import frc.robot.commands.OperateCargo;
import frc.robot.subsystems.CargoUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabAndShoot extends SequentialCommandGroup {
  /** Creates a new GrabNShoot. */
  public GrabAndShoot(DriveUtil du, CargoUtil cu, ShotUtil su) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        //Drives forward while intaking ball
      new ParallelDeadlineGroup( 
        new DriveForDistanceNoPID(du, 60, true),
        new AutoIntake(cu)
      ),
      //Turns to face the goal
      new TurnForAngle(du, 170),
      new DriveForDistanceNoPID(du, 80, false),
      // // //Instead of doing precise turns, driving into it for enough time will automatically line us up
      new ParallelCommandGroup(
        new DriveForTime(du, 2, Constants.AUTO_DRIVE_SPEED),
        new AutoShoot(su)
      ),
      // // //Get exact right positioning for shooting
      new AutoSpinUp(cu)
    );
  }
}
