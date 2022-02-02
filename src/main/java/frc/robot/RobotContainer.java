// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.OperateDrive;
import frc.robot.commands.OperateColorSensor;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.CargoUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveUtil driveUtil = new DriveUtil();
  private final CargoUtil cargoUtil = new CargoUtil();

  private final OperateDrive operateDrive = new OperateDrive(driveUtil);
  private final OperateColorSensor operateColorSensor = new OperateColorSensor(cargoUtil);

  public static XboxController operator;

  public static SendableChooser<Byte> driveType;
  public static SendableChooser<Byte> noobMode;
  public final static Byte arcade = 0;
  public final static Byte tank = 1;
  public final static Byte curvature = 2;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    operator = new XboxController(Constants.XBOX);

    driveType = new SendableChooser<>();
    driveType.setDefaultOption("Arcade", arcade);
    driveType.addOption("Tank", tank);
    driveType.addOption("Curvature", curvature);
    SmartDashboard.putData("Drive Type", driveType);    
    
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  private void configureDefaultCommands(){
    driveUtil.setDefaultCommand(operateDrive);
    //cargoUtil.setDefaultCommand(operateColorSensor);
  }

  public static double getLeftXboxX(){
    return operator.getLeftX();
  }

  public static double getLeftXboxY(){
    return operator.getLeftY();
  }

  public static double getRightXboxX(){
    return operator.getRightX();
  }

  public static double getRightXboxY(){
    return operator.getRightY();
  }

  public static double getLeftXboxTrigger(){
    return operator.getLeftTriggerAxis();
  }

  public static double getRightXboxTrigger(){
    return operator.getRightTriggerAxis();
  }

}
