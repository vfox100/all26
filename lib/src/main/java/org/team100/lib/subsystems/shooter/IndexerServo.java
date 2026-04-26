package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Indexer using continuous-rotation servo or PWM controller. */
public class IndexerServo extends SubsystemBase {
    private final PWM m_pwm;
    private final DoubleLogger m_log_dutyCycle;

    public IndexerServo(LoggerFactory parent, int channel) {
        LoggerFactory logger = parent.type(this);
        m_log_dutyCycle = logger.doubleLogger(Level.TRACE, "duty cycle");
        m_pwm = new PWM(channel);
    }

    public void set(double dutyCycle) {
        m_pwm.setSpeed(-1.0 * dutyCycle);
        m_log_dutyCycle.log(() -> dutyCycle);
    }

    public void stop() {
        set(0);
    }

    public Command feed() {
        return run(() -> set(1));
    }
}
