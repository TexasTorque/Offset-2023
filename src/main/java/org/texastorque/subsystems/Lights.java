package org.texastorque.subsystems;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class Lights extends TorqueSubsystem implements Subsystems {
    private static volatile Lights instance;

    private final AddressableLED leds;
    private final AddressableLEDBuffer buff;
    private Color mainColor = Color.kRed, alternateColor = Color.kGreen;

    private static final int LENGTH = 6;

    private static final double BLINK_HZ = 3; 

    public void set(final Color main, final Color alternate) {
        this.mainColor = main;
        this.alternateColor = alternate;
    }

    private Lights() {
        leds = new AddressableLED(0);
        leds.setLength(LENGTH);
        buff = new AddressableLEDBuffer(LENGTH);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        leds.start();
    }

    @Override
    public final void update(final TorqueMode mode) {

        if (alternateColor.equals(SOLID)) {
            setColor(mainColor);
        } else {
            final double timestamp = TorqueUtil.time();
            final boolean on = (Math.floor(timestamp * BLINK_HZ) % 2 == 1);
            setColor(on ? alternateColor : mainColor);
        }
    }

    public static final Color OFF = Color.kBlanchedAlmond;
    public static final Color SOLID = Color.kBurlywood;
    public static final Color ALLIANCE = DriverStation.getAlliance() == Alliance.Blue ? Color.kBlue : Color.kRed;

    private final void setColor(final Color color) {
        if (color.equals(OFF)) {
            leds.stop();
            return;
        }

        for (int i = 0; i < LENGTH; i++)
            buff.setLED(i, color);
        leds.setData(buff);

        leds.start();
    }


    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }
}
