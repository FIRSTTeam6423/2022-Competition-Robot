// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimbUtil extends SubsystemBase{
    //Solenoid
    private Solenoid grabber;
    private boolean state;

    public ClimbUtil(){
        grabber = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        state = false;
    }

    public void stateIsTrue(){
        state = true;
    }

    public void stateIsFalse(){
        state = false;
    }

    public boolean returnState(){
        return state;
    }

    public void raiseArm(){
        grabber.set(true);
    }
}
