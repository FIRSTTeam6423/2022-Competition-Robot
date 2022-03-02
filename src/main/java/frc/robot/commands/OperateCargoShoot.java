package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CargoUtil;
import frc.robot.RobotContainer;
import frc.robot.enums.CargoState;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class OperateCargoShoot extends CommandBase{
    CargoUtil cu;
    boolean done = false;
    public OperateCargoShoot(CargoUtil cu){
        this.cu = cu;
        addRequirements(this.cu);
    }

    @Override
    public void initialize() {
      //cu.setState(CargoState.SPINUP);
      //SmartDashboard.putString("Shooter State", "Spin up");
    }

    @Override
    public void execute() {
      double rpm = cu.getShooterRPM();
      if(rpm >= Constants.SHOOTER_RPM - Constants.SHOOTER_RPM_DEADBAND && 
        rpm <= Constants.SHOOTER_RPM + Constants.SHOOTER_RPM_DEADBAND)
      {
        if (RobotContainer.getOperatorRightBumper()){
          cu.setState(CargoState.SHOOT);
          done = true;
          //SmartDashboard.putString("Shooter State", "Shoot");
        }
      }
      cu.OperateCargo();
    }

    @Override
    public void end(boolean interrupted) {
        cu.setState(CargoState.IDLE);
        //SmartDashboard.putString("Shooter State", "idle");
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
