package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveUtil;
import frc.robot.Constants;

public class DriveForDistance extends CommandBase{
    DriveUtil driveUtil;
    double distanceToDrive;
    private double encoderSetpoint;
    private boolean done;
    
    public DriveForDistance(DriveUtil du, double distanceToDrive) {
        this.driveUtil = du;
        this.distanceToDrive = distanceToDrive * Constants.TICKS_PER_INCH;
        addRequirements(this.driveUtil);
    }

    @Override
    public void initialize() {
        done = false;
        encoderSetpoint = driveUtil.getPosition() + distanceToDrive;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!driveUtil.getMoving() && 
            driveUtil.getLeftPosition() > encoderSetpoint - Constants.DRIVER_DEADBAND && 
            driveUtil.getLeftPosition() < encoderSetpoint + Constants.DRIVER_DEADBAND && 
            driveUtil.getRightPosition() > encoderSetpoint - Constants.DRIVER_DEADBAND && 
            driveUtil.getRightPosition() < encoderSetpoint + Constants.DRIVER_DEADBAND) {
                driveUtil.stopDistance();
                done = true;
                return;
            }
            
        driveUtil.operateDistance(encoderSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        driveUtil.stopDistance();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
