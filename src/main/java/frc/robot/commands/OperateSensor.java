package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SensorUtil;

public class OperateSensor extends CommandBase{
    private SensorUtil sensorUtil;
    
    /**Creates a new OperateCargo */
    public OperateSensor(SensorUtil du){
        // Use addRequirements() here to declare subsystem dependencies.
        this.sensorUtil = du;
        addRequirements(this.sensorUtil);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        sensorUtil.detectBallColor();
        sensorUtil.detectBall();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}