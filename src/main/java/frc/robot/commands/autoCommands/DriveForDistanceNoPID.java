package frc.robot.commands.autoCommands;

import frc.robot.subsystems.DriveUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DriveForDistanceNoPID extends CommandBase{
    private DriveUtil driveUtil;
    private double encoderSetpoint;
    private boolean forward;
    private boolean done;
    private boolean slowdown;
    
    public DriveForDistanceNoPID(DriveUtil du, double distanceToDrive, boolean slowdown) {
        this.driveUtil = du;
        this.slowdown = slowdown;
        this.encoderSetpoint = distanceToDrive * Constants.TICKS_PER_INCH;
        addRequirements(this.driveUtil);
    }

    @Override
    public void initialize() {
        done = false;

        driveUtil.resetEncoder();

        //If the distance is negative the robot will go backwards
        if (encoderSetpoint > 0){
            forward = true;
        } else {
            forward = false;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Checks if robot has gone the distance 
        if (Math.abs(driveUtil.getLeftPosition()) >= Math.abs(encoderSetpoint)){
            driveUtil.tankDrive(0, 0);
            done = true;
            return;
        }

        if (forward){
            //Checks if robot is in slowodown range
            if (slowdown && Math.abs(driveUtil.getLeftPosition()) >= (Math.abs(encoderSetpoint) - Math.abs(Constants.AUTO_DRIVE_SLOWDOWN_RANGE * Constants.TICKS_PER_INCH))){
                driveUtil.tankDrive(Constants.AUTO_DRIVE_SPEED * Constants.AUTO_DRIVE_SPEED_DAMPENING, -Constants.AUTO_DRIVE_SPEED * Constants.AUTO_DRIVE_SPEED_DAMPENING);
            } else {
                driveUtil.tankDrive(Constants.AUTO_DRIVE_SPEED, -Constants.AUTO_DRIVE_SPEED);
            }
        } else {
            //Checks if robot is in slowodown range
            if (slowdown && Math.abs(driveUtil.getLeftPosition()) >= (Math.abs(encoderSetpoint) - Math.abs(Constants.AUTO_DRIVE_SLOWDOWN_RANGE * Constants.TICKS_PER_INCH))){
                driveUtil.tankDrive(-Constants.AUTO_DRIVE_SPEED * Constants.AUTO_DRIVE_SPEED_DAMPENING, Constants.AUTO_DRIVE_SPEED * Constants.AUTO_DRIVE_SPEED_DAMPENING);
            } else {
                driveUtil.tankDrive(-Constants.AUTO_DRIVE_SPEED, Constants.AUTO_DRIVE_SPEED);
            }
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
