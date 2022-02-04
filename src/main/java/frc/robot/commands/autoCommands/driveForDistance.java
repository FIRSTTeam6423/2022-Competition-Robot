package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveUtil;

public class driveForDistance extends CommandBase{
    DriveUtil driveUtil;
    double distanceToDrive;

    
    public driveForDistance(DriveUtil du, double distanceToDrive) {
        this.distanceToDrive = distanceToDrive;
        this.driveUtil = du;
        addRequirements(this.driveUtil);

    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveUtil.tankDrive(driveUtil.autoMoveDistance(5), -driveUtil.autoMoveDistance(5));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
