package frc.robot.commands.autoCommands;

import frc.robot.subsystems.DriveUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class driveForDistanceNoPID extends CommandBase{
    DriveUtil driveUtil;
    double distanceToDrive;
    private double encoderSetpoint;
    
    public driveForDistanceNoPID(DriveUtil du, double distanceToDrive) {
        this.driveUtil = du;
        this.distanceToDrive = distanceToDrive * Constants.TICKS_PER_INCH;
        addRequirements(this.driveUtil);
    }

    @Override
    public void initialize() {
        encoderSetpoint = driveUtil.getPosition() + distanceToDrive;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveUtil.tankDrive(0.2, 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        driveUtil.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return !driveUtil.getMoving() && driveUtil.getleftPosition() > encoderSetpoint - Constants.DRIVER_DEADBAND && driveUtil.getleftPosition() < encoderSetpoint + Constants.DRIVER_DEADBAND
        && driveUtil.getrightPosition() > encoderSetpoint - Constants.DRIVER_DEADBAND && driveUtil.getrightPosition() < encoderSetpoint + Constants.DRIVER_DEADBAND;
    }
}
