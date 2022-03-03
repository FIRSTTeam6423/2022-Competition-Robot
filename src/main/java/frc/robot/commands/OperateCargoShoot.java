package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CargoUtil;
import frc.robot.util.CargoState;
import frc.robot.RobotContainer;
import frc.robot.Constants;


public class OperateCargoShoot extends CommandBase{
    CargoUtil cu;
    boolean done = false;
    public OperateCargoShoot(CargoUtil cu){
        this.cu = cu;
        addRequirements(this.cu);
    }

    @Override
    public void initialize() {
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
        }
      }
      cu.OperateCargo();
    }

    @Override
    public void end(boolean interrupted) {
        cu.setState(CargoState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
