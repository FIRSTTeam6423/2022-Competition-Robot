package frc.robot.commands.autoCommands;

import frc.robot.subsystems.DriveUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putString("Drive States", "Resetting");
        done = false;

        driveUtil.resetEncoder();

        if (encoderSetpoint > 0){
            forward = true;
        } else {
            forward = false;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(driveUtil.getLeftPosition()) >= Math.abs(encoderSetpoint)){
            driveUtil.tankDrive(0, 0);
            SmartDashboard.putString("Drive States", "Stopped");
            done = true;
            return;
        }

        SmartDashboard.putNumber("Setpoint", encoderSetpoint);
        SmartDashboard.putNumber("Position", Math.abs(driveUtil.getLeftPosition()));

        if (forward){
            if (Math.abs(driveUtil.getLeftPosition()) >= (Math.abs(encoderSetpoint) - Math.abs(Constants.AUTO_DRIVE_SLOWDOWN_RANGE * Constants.TICKS_PER_INCH))){
                driveUtil.tankDrive(Constants.AUTO_DRIVE_SPEED * Constants.AUTO_DRIVE_SPEED_DAMPENING, -Constants.AUTO_DRIVE_SPEED * Constants.AUTO_DRIVE_SPEED_DAMPENING);
                SmartDashboard.putString("Drive States", "SLowing down");
            } else {
                driveUtil.tankDrive(Constants.AUTO_DRIVE_SPEED, -Constants.AUTO_DRIVE_SPEED);
                SmartDashboard.putString("Drive States", "Full Speed");
            }
        } else {
            if (Math.abs(driveUtil.getLeftPosition()) >= (Math.abs(encoderSetpoint) - Math.abs(Constants.AUTO_DRIVE_SLOWDOWN_RANGE * Constants.TICKS_PER_INCH))){
                driveUtil.tankDrive(-Constants.AUTO_DRIVE_SPEED * Constants.AUTO_DRIVE_SPEED_DAMPENING, Constants.AUTO_DRIVE_SPEED * Constants.AUTO_DRIVE_SPEED_DAMPENING);
                SmartDashboard.putString("Drive States", "SLowing down");
            } else {
                driveUtil.tankDrive(-Constants.AUTO_DRIVE_SPEED, Constants.AUTO_DRIVE_SPEED);
                SmartDashboard.putString("Drive States", "Full Speed");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveUtil.tankDrive(0, 0);
        SmartDashboard.putString("Drive States", "Stopped");
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
