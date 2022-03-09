// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ClimbState;

public class ClimbUtil extends SubsystemBase{
    //Solenoid
    private Solenoid grabber;
    private Compressor pcmCompressor;

    /**
     * I created an enum class because it is cleaner than using a boolean.
     * Boolean is easier but sometimes folks forget that true means out and false means back.
     * With an enum we are explicitely typed and the coding is more understandable.
     * 
     * Always good to set the default state and to make sure it complies with robot 
     * start position constraints.
     */
    private ClimbState state = ClimbState.ARM_BACK;

    public ClimbUtil(){
        grabber = new Solenoid(10, PneumaticsModuleType.CTREPCM, 0);
        pcmCompressor = new Compressor(10, PneumaticsModuleType.CTREPCM);
        pcmCompressor.enableDigital();
    }

    public void toggleArmState(){
        if(state != ClimbState.ARM_OUT){
            state = ClimbState.ARM_OUT;
        }else{
            state = ClimbState.ARM_BACK;
        }
    }

    public void setState(ClimbState input){
        state = input;
    }

    public ClimbState getPistonState(){
        return state;
    }

    public void operateArm(){
        if(state == ClimbState.ARM_BACK){
            grabber.set(false);
        }
        if(state == ClimbState.ARM_OUT){
            grabber.set(true);
        }
    }

    @Override
    public void periodic(){
    }
}
