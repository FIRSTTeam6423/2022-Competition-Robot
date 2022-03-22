package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CargoUtil;
import frc.robot.util.CargoState;

public class AutoIntake extends CommandBase{
    CargoUtil cu;
    boolean done = false;

    public AutoIntake(CargoUtil cu){
        this.cu = cu;
        addRequirements(this.cu);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        cu.operateBallMagnet();
        // cu.operateLowIndexer();
        if (cu.detectLowerBall()){
            cu.stopBallMagnet();
            // cu.stopLowIndexer();
            done = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}
