package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CargoUtil;
import frc.robot.RobotContainer;
import frc.robot.enums.CargoState;

public class OperateCargoIntake extends CommandBase{
    CargoUtil cu;

    public OperateCargoIntake(CargoUtil cu){
        this.cu = cu;
        addRequirements(this.cu);
    }

    @Override
    public void initialize() {
        cu.setState(CargoState.INTAKE);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.getOperatorAButton()){
            cu.setState(CargoState.INDEX);
        
        } 
        if(RobotContainer.getOperatorXButton() && cu.detectLowerBall()) {
            cu.setState(CargoState.SPIT);
        } else if (RobotContainer.getOperatorXButton() && !cu.detectLowerBall()){
            cu.setState(CargoState.INTAKE);
        }
        if (RobotContainer.getOperatorYButton()){
            cu.setState(CargoState.IDLE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        cu.setState(CargoState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
