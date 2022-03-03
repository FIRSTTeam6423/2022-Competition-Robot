// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class DPadButton extends Button {
  private final GenericHID m_joystick;
  private final int angle;

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public DPadButton(GenericHID joystick, int angle) {
    requireNonNullParam(joystick, "joystick", "JoystickButton");

    m_joystick = joystick;
    this.angle = angle;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    return m_joystick.getPOV(0) == angle;
  }
}
