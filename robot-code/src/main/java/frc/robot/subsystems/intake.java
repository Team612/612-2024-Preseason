package main.java.frc.robot.subsystems;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import Compressor;



public class intake extends SubsystemBase{
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
    boolean enabled = pcmCompressor.enabled();
    boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    double current = pcmCompressor.getCompressorCurrent();
    Solenoid exampleSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    Solenoid exampleSolenoidPH = new Solenoid(PneumaticsModuleType.REVPH, 1);

    Solenoid exampleSingle = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    DoubleSolenoid exampleDouble = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);{

// Initialize the DoubleSolenoid so it knows where to start.  Not required for single solenoids.
    // exampleDouble.set(kReverse);
    if (m_controller.getYButtonPressed()) {
        exampleSingle.toggle();
        exampleDouble.toggle();
    }
    // in this case, 250(V/5)-25
    // the scale parameter in the AnalogPotentiometer constructor is scaled from 1 instead of 5,
    // so if r is the raw AnalogPotentiometer output, the pressure is 250r-25
    double scale = 250, offset = -25;
    AnalogPotentiometer pressureTransducer = new AnalogPotentiometer(/* the AnalogIn port*/ 2, scale, offset);

    // scaled values in psi units
    double psi = pressureTransducer.get();
    double current2 = phCompressor.getPressure();
        DoubleSolenoid exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
        DoubleSolenoid exampleDoublePH = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 4, 5);
        exampleDoublePCM.set(kOff);
        exampleDoublePCM.set(kForward);
        exampleDoublePCM.set(kReverse);
}
}

