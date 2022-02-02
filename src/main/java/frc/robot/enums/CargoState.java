package frc.robot.enums;

public enum CargoState {
    IDLE,INTAKE,INDEX,SPINUP,SHOOT;
    /**Idle - No motors are running
     * Intake - Ball magnet and Low Indexer are running
     * Index - Low Indexer is running
     * Spinup - High indexer in running and Shooter is speeding up
     * Shoot - Shooter is at spead and High Indexer is running
     */
}
