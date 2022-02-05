package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveUtil;
import frc.robot.Constants;

public class driveForDistance extends CommandBase{
    DriveUtil driveUtil;
    double distanceToDrive;
    private double encoderSerpoint;
    
    public driveForDistance(DriveUtil du, double distanceToDrive) {
        this.driveUtil = du;
        this.distanceToDrive = distanceToDrive * Constants.TICKS_PER_INCH;
        addRequirements(this.driveUtil);
    }

    @Override
    public void initialize() {
        encoderSerpoint = driveUtil.getPosition() + distanceToDrive;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveUtil.operateDistance(distanceToDrive);
    }

    @Override
    public void end(boolean interrupted) {
        driveUtil.stopDistance();
    }

    @Override
    public boolean isFinished() {
        return driveUtil.getPosition() > encoderSerpoint;
    }
}
