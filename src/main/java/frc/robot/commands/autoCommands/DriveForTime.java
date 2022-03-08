// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveUtil;

public class DriveForTime extends CommandBase {
    DriveUtil driveUtil;
    Timer timer;
    double timeToDrive;
    double speed;
    boolean done;
    
    public DriveForTime(DriveUtil du, double timeToDrive, double speed) {
        this.timeToDrive = timeToDrive;
        this.speed = speed;
        this.driveUtil = du;
        addRequirements(this.driveUtil);

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
        done = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Checks if the robot has fully turned
        driveUtil.tankDrive(speed, -speed);
        if (timer.get() > timeToDrive){
            done = true;
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
