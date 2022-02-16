// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.OperateDrive;
import frc.robot.commands.OperateCargoIntake;
import frc.robot.commands.OperateCargoShoot;
import frc.robot.commands.autoCommands.DriveForTime;
import frc.robot.subsystems.DriveUtil;
import frc.robot.subsystems.CargoUtil;
import frc.robot.subsystems.ClimbUtil;
import frc.robot.commands.autoCommands.DrivBoxPattern;
import frc.robot.commands.autoCommands.DriveForDistance;
import frc.robot.commands.autoCommands.DriveForDistanceNoPID;

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
  private final ClimbUtil climbUtil = new ClimbUtil();

  private final OperateDrive operateDrive = new OperateDrive(driveUtil);
  private final OperateCargoIntake operateCargoIntake = new OperateCargoIntake(cargoUtil);
  private final OperateCargoShoot operateCargoShoot = new OperateCargoShoot(cargoUtil);

  private static XboxController driver;
  private static XboxController operator;

  /**
   * Added a new object - JoystickButton
   * This one is used to Toggle the Climb Arm out and back.
   */
  private JoystickButton toggleClimb;


  public static SendableChooser<Byte> driveType;
  public static SendableChooser<Byte> noobMode;
  public static SendableChooser<String> teamColorChooser; 
  public final static Byte arcade = 0;
  public final static Byte tank = 1;
  public final static Byte curvature = 2;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    operator = new XboxController(Constants.XBOX_OPERATOR);
    driver = new XboxController(Constants.XBOX_DRIVER);

    driveType = new SendableChooser<>();
    driveType.setDefaultOption("Arcade", arcade);
    driveType.addOption("Tank", tank);
    driveType.addOption("Curvature", curvature);
    SmartDashboard.putData("Drive Type", driveType);
    
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();

    autoChooser.addOption("Drive 24 Inches Forward No PID", new DriveForDistanceNoPID(driveUtil, 24));
    autoChooser.setDefaultOption("Drive 60 Inches Forward No PID", new DriveForDistanceNoPID(driveUtil, 60));
    autoChooser.addOption("Drive 120 Inches Forward No PID", new DriveForDistanceNoPID(driveUtil, 120));

    autoChooser.addOption("Drive 24 Inches Backward No PID", new DriveForDistanceNoPID(driveUtil, -24));
    autoChooser.addOption("Drive 60 Backward Forward No PID", new DriveForDistanceNoPID(driveUtil, -60));
    autoChooser.addOption("Drive 120 Inches Backward No PID", new DriveForDistanceNoPID(driveUtil, -120));

    teamColorChooser = new SendableChooser<String>();
    teamColorChooser.addOption("Red", "RED");
    teamColorChooser.addOption("Blue", "BLUE");
    SmartDashboard.putData("Team Color", teamColorChooser);

    autoChooser.addOption("Drive in box", new DrivBoxPattern(driveUtil, 36));

    SmartDashboard.putData("Autonomous Command", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**
     * Actually added code here this time.
     * First you instantiate your Button (toggleClimb).
     * Although you use the generic JoystickButton class here
     * be careful.  The second variable in the constructor does all the work.
     * I used the Button object in this class:
     *    edu.wpi.first.wpilibj.XboxController.Button;
     * However there are Button classes for PS4 game controllers and more!!!!
     * Careful what you choose!
     * 
     */
    toggleClimb = new JoystickButton(operator, Button.kY.value);

    /**
     * Could have done this any number of ways, a real command or an instant command.
     * I went with InstantCommand, just as an example.  It will work.  Much more lightweight
     * than a full blown Command class given we just want to toggle the state of something.
     * 
     * You will notice that we got rid of the OperateCommand() command class as it is
     * not needed in the case of an InstantCommand().
     * 
     */
    toggleClimb.whenPressed(new InstantCommand(() -> climbUtil.toggleArmState(), climbUtil));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

  private void configureDefaultCommands(){
    driveUtil.setDefaultCommand(operateDrive);
    cargoUtil.setDefaultCommand(operateCargoIntake);
    cargoUtil.setDefaultCommand(operateCargoShoot);
  }

  public static double getDriverLeftXboxX(){
    return driver.getLeftX();
  }

  public static double getDriverLeftXboxY(){
    return driver.getLeftY();
  }

  public static double getDriverRightXboxX(){
    return driver.getRightX();
  }

  public static double getDriverRightXboxY(){
    return driver.getRightY();
  }

  public static double getDriverLeftXboxTrigger(){
    return driver.getLeftTriggerAxis();
  }

  public static double getDriverRightXboxTrigger(){
    return driver.getRightTriggerAxis();
  }

  public static boolean getDriverAButton(){
    return driver.getAButton();
  }

  public static boolean getDriverBButton(){
    return driver.getBButton();
  }

  public static boolean getDriverXButton(){
    return driver.getXButton();
  }

  public static boolean getDriverYButton(){
    return driver.getYButton();
  }

  public static boolean getDriverLeftBumper(){
    return driver.getLeftBumper();
  }

  public static boolean getDriverRightBumper(){
    return driver.getRightBumper();
  }

  public static boolean getDriverLeftStickButton(){
    return driver.getLeftStickButton();
  }  

  public static boolean getDriverRightStickButton(){
    return driver.getRightStickButton();
  } 


  public static double getOperatorLeftXboxX(){
    return operator.getLeftX();
  }

  public static double getOperatorLeftXboxY(){
    return operator.getLeftY();
  }

  public static double getOperatorRightXboxX(){
    return operator.getRightX();
  }

  public static double getOperatorRightXboxY(){
    return operator.getRightY();
  }

  public static double getOperatorLeftXboxTrigger(){
    return operator.getLeftTriggerAxis();
  }

  public static double getOperatorRightXboxTrigger(){
    return operator.getRightTriggerAxis();
  }

  public static boolean getOperatorAButton(){
    return operator.getAButton();
  }

  public static boolean getOperatorBButton(){
    return operator.getBButton();
  }

  public static boolean getOperatorXButton(){
    return operator.getXButton();
  }

  public static boolean getOperatorYButton(){
    return operator.getYButton();
  }

  public static boolean getOperatorLeftBumper(){
    return operator.getLeftBumper();
  }

  public static boolean getOperatorRightBumper(){
    return operator.getRightBumper();
  }

  public static boolean getOperatorLeftStickButton(){
    return operator.getLeftStickButton();
  }  

  public static boolean getOperatorRightStickButton(){
    return operator.getRightStickButton();
  }  

  public static String getTeamColor(){
    return teamColorChooser.getSelected();
  }
}
