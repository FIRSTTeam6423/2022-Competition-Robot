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
        // This method in DriveUtil sets the position of the encoders to 0.
        // This assures you the encoder values start from 0, and you don't have to adjust for the starting position.
        // Determine whether we're supposed to drive forward or backward based on the sign of the targetTicks value.
        // If the value is positive, drive forward. If negative, drive backward!
        driveUtil.resetGyro();

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
        // If the absolute value of the left encoder is greater than or equal to the absolute value of our target distance...
        // we're done! Set `done = true` and turn the motors off.
        // We only need to check the left encoder since we're driving straight.
        // If you're turning, you would use the gyro to measure that properly.
        angle = driveUtil.getGyroYaw();

        if (right) {
            // Drive the robot using a constant speed set in Constants
            // It's useful to use a Constant value since you can easily change it while testing!
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
            return;
        }
        if (right) {
            // Drive the robot using a constant speed set in Constants
            // It's useful to use a Constant value since you can easily change it while testing!
            if (angle < 0){
                angle += 360;
            }

            if (Math.abs(angle) >= Math.abs(targetAngle) - Constants.AUTO_TURN_SLOWDOWN_RANGE){
                driveUtil.tankDrive(Constants.AUTO_TURN_SPEED * Constants.AUTO_TURN_SPEED_DAMPENING, Constants.AUTO_TURN_SPEED * Constants.AUTO_TURN_SPEED_DAMPENING);
            } else {
                driveUtil.tankDrive(Constants.AUTO_TURN_SPEED, Constants.AUTO_TURN_SPEED);
            }
        } else {
            if (angle > 0){
                angle -= 360;
            }

            //If we're driving backwards, multiply the Constants value by -1!
            if (Math.abs(angle) >= Math.abs(targetAngle) - Constants.AUTO_TURN_SLOWDOWN_RANGE){
                driveUtil.tankDrive(-Constants.AUTO_TURN_SPEED * Constants.AUTO_TURN_SPEED_DAMPENING, -Constants.AUTO_TURN_SPEED * Constants.AUTO_TURN_SPEED_DAMPENING);
            } else {
                driveUtil.tankDrive(-Constants.AUTO_TURN_SPEED, -Constants.AUTO_TURN_SPEED);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Set the motors off in the end() method as a safety. Always turn motors off when you're done!
        driveUtil.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        // Generally it's clearer and easier for debugging to `return done` in isFinished rather than doing all your distance checks here.
        // The value of `done` could be printed to the SmartDashboard if the command isn't behaving properly, for example.
        // If you didn't have that done boolean, it can complicate debugging.
        return done;
    }
}