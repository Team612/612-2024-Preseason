package main.java.frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.lang.AutoCloseable;

public class motorthingy extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    public CANSparkMax boomMax;
    public motorthingy() {
        boomMax = new CANSparkMax(SparkPorts.tele_arm, MotorType.kBrushless);
        oomMax.setIdleMode(IdleMode.kBrake);
    }
  
    public CommandBase settyTheSpeed(double speed) {
        boomMax.set(speed);
    }
  

    public boolean exampleCondition() {

    }
  
    @Override
    public void periodic() {

    }
  
    @Override
    public void simulationPeriodic() {

    }
}

