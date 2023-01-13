package org.texastorque.subsystems;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public final class Lights extends TorqueSubsystem implements Subsystems {
    private static volatile Lights instance;

    private final AddressableLED leds;
    private final AddressableLEDBuffer buff;
    private Color lastColor, color;
    private boolean blink = false, on = true, lastOn = false;

    private static int LENGTH = 6;

    public void set(final Color color, final boolean blink) {
        this.color = color;
        this.blink = blink;
    }

    private Lights() {
        color = Color.kGreen;
        lastColor = Color.kRed;

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
        setColor();

        if (!blink) return;

        final double timestamp = TorqueUtil.time();
        on = Math.floor(timestamp) % 2 == 0;
        if (lastOn != on) {
            if (on) leds.start();
            else leds.stop();
            lastOn = on;
        }
    }

    private final void setColor() {
        if (color.equals(lastColor)) return;

        for (int i = 0; i < LENGTH; i++)
            buff.setLED(i, color);
        leds.setData(buff);

        lastColor = color;
    }


    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }
}
