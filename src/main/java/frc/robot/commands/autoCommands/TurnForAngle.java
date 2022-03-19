package frc.robot.commands.autoCommands;

import frc.robot.subsystems.DriveUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class TurnForAngle extends CommandBase {
    private DriveUtil driveUtil;
    private double targetAngle;
    private boolean right;
    private boolean done;
    private double angle;

    public TurnForAngle(DriveUtil driveUtil, double angleToTurn) {
        this.driveUtil = driveUtil;
        this.targetAngle = angleToTurn;
        addRequirements(this.driveUtil);
    }

    @Override
    public void initialize() {
        driveUtil.resetGyro();

        // If angle is positive turn right
        // If angle is negative turn left
        if (targetAngle < 0) {
            right = false;
        } else {
            right = true;
        }

        done = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        angle = driveUtil.getGyroYaw();

        if (right) {
            // Converts the gyro angles to 0->360 instead of 0->180 & 0->-180
            if (angle < 0){
                angle += 360;
            }
        } else {
            if (angle > 0){
                angle -= 360;
            }
        }

        if (Math.abs(angle) >= Math.abs(targetAngle)) {
            driveUtil.tankDrive(0, 0);
            done = true;
            System.out.println(angle);
            return;
        }
        if (right) {
            // Checks if in slowdown range
            if (Math.abs(angle) >= Math.abs(targetAngle) - Constants.AUTO_TURN_SLOWDOWN_RANGE){
                driveUtil.tankDrive(Constants.AUTO_TURN_SPEED * Constants.AUTO_TURN_SPEED_DAMPENING, Constants.AUTO_TURN_SPEED * Constants.AUTO_TURN_SPEED_DAMPENING);
            } else {
                driveUtil.tankDrive(Constants.AUTO_TURN_SPEED, Constants.AUTO_TURN_SPEED);
            }
        } else {
            // Checks if in slowdown range
            if (Math.abs(angle) >= Math.abs(targetAngle) - Constants.AUTO_TURN_SLOWDOWN_RANGE){
                driveUtil.tankDrive(-Constants.AUTO_TURN_SPEED * Constants.AUTO_TURN_SPEED_DAMPENING, -Constants.AUTO_TURN_SPEED * Constants.AUTO_TURN_SPEED_DAMPENING);
            } else {
                driveUtil.tankDrive(-Constants.AUTO_TURN_SPEED, -Constants.AUTO_TURN_SPEED);
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