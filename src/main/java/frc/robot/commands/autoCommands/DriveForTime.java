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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveUtil.tankDrive(speed, -speed);
        System.out.println("Drive for time works");
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return timer.get() > timeToDrive;
    }
}
