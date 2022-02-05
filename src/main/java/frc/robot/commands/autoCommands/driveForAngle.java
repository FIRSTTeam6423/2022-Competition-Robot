
package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveUtil;

public class driveForAngle extends CommandBase{
    DriveUtil driveUtil;
    double angleToTurn;
    private double angleSetpoint;
    
    public driveForAngle(DriveUtil du, double angleToTurn) {
        this.driveUtil = du;
        this.angleToTurn = angleToTurn;
        addRequirements(this.driveUtil);
    }

    @Override
    public void initialize() {
        angleSetpoint = driveUtil.getHeading() + angleToTurn;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveUtil.tankDrive(1, 0);
    }

    @Override
    public void end(boolean interrupted) {
        driveUtil.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return driveUtil.getHeading() > angleSetpoint;
    }
}