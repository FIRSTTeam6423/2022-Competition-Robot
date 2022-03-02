// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.ClimbState;

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
        grabber = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        pcmCompressor = new Compressor(10, PneumaticsModuleType.CTREPCM);
        pcmCompressor.enableDigital();
    }

    /**
     * We are doing the bulk of the work in the Util class which is 
     * the right way to do this.  The Util class should manage its own
     * state and give helper methods to commands and other classes so that they can 
     * discover the state of the subsystem.
     * 
     * Here we are simply toggling between two states.  This can still 
     * be done with more than one state but it becomes a tad more complicated to code.
     * 
     * You may want to try expressly setting the desired state at the Command
     * and work from there in cases where you have 3+ plausible states for a subsystem.
     */
    public void toggleArmState(){
        if(state != ClimbState.ARM_OUT){
            state = ClimbState.ARM_OUT;
        }else{
            state = ClimbState.ARM_BACK;
        }
    }

    /**
     * Simple helper class to determine the state of the subsystem.
     * In this example we are just running it out to the SmartDashboard
     * for folks to see.
     * 
     */
    public ClimbState getPistonState(){
        return state;
    }

    /**
     * This is where the actual action takes place.
     * This method is called continuously from the 
     * subsystem periodic() method.  It is checking the
     * state and making sure it is so.
     * 
     * Although this repeats over and over again, when dealing
     * with pneumatics it works and it is a no harm no foul approach.
     */
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
        /**
         * Here is where we are checking the desired state and making sure it is so.
         * Additionally, we are sending state to the SmartDashboard.
         */
        //operateArm();
        SmartDashboard.putString("Climb State :: ", getPistonState().toString());
        SmartDashboard.putBoolean("Compressor", pcmCompressor.enabled());
        SmartDashboard.putBoolean("Pressure Switch", pcmCompressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Solonoid", grabber.get());
    }
}
