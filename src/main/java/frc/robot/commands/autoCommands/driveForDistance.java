package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveUtil;

public class driveForDistance extends CommandBase{
    DriveUtil driveUtil;
    double distanceToDrive;
    private double encoderSetpoint;
    
    public driveForDistance(DriveUtil du, double distanceToDrive) {
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
        driveUtil.operateDistance(encoderSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        driveUtil.stopDistance();
    }

    @Override
    public boolean isFinished() {
        /**
         * This command won't work since PID logic was commented out.
         */
        return true;
        // return !driveUtil.getMoving() && driveUtil.getLeftEncoder() > encoderSetpoint - Constants.DRIVER_DEADBAND && driveUtil.getLeftEncoder() < encoderSetpoint + Constants.DRIVER_DEADBAND
        // && driveUtil.getRightEncoder() > encoderSetpoint - Constants.DRIVER_DEADBAND && driveUtil.getRightEncoder() < encoderSetpoint + Constants.DRIVER_DEADBAND;
    }
}
