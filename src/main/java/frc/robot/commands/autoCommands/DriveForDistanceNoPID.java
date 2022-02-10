package frc.robot.commands.autoCommands;

import frc.robot.subsystems.DriveUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DriveForDistanceNoPID extends CommandBase{
    DriveUtil driveUtil;
    private double encoderSetpoint;
    private boolean forward;
    private boolean done;
    
    public DriveForDistanceNoPID(DriveUtil du, double distanceToDrive) {
        this.driveUtil = du;
        this.encoderSetpoint = distanceToDrive * Constants.TICKS_PER_INCH;
        addRequirements(this.driveUtil);
    }

    @Override
    public void initialize() {
        done = false;

        driveUtil.resetEndconder();

        if (encoderSetpoint > 0){
            forward = true;
        } else {
            forward = false;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(driveUtil.getleftPosition()) >= Math.abs(encoderSetpoint) - Constants.DRIVER_DEADBAND
            && Math.abs(driveUtil.getleftPosition()) <= Math.abs(encoderSetpoint) + Constants.DRIVER_DEADBAND){
            driveUtil.tankDrive(0, 0);
            done = true;
            return;
        }

        if (forward){
            driveUtil.tankDrive(Constants.AUTO_DRIVE_SPEED, -Constants.AUTO_DRIVE_SPEED);
        } else {
            driveUtil.tankDrive(-Constants.AUTO_DRIVE_SPEED, Constants.AUTO_DRIVE_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveUtil.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
