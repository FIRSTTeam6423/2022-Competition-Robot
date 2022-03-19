package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CargoUtil;
import frc.robot.util.CargoState;

public class OperateCargo extends CommandBase{
    CargoUtil cu;

    public OperateCargo(CargoUtil cu){
        this.cu = cu;
        addRequirements(this.cu);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        cu.OperateCargo();
    }

    @Override
    public void end(boolean interrupted) {
        cu.setState(CargoState.IDLE);
        cu.OperateCargo();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}