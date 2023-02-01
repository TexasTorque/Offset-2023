package org.texastorque.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public final class Lights extends TorqueSubsystem implements Subsystems {
    private static volatile Lights instance;

    private final AddressableLED leds;
    private final AddressableLEDBuffer buff;

    private static final int LENGTH = 20;

    private List<LightAction> actions = new ArrayList<>();

    private Lights() {
        leds = new AddressableLED(0);
        leds.setLength(LENGTH);
        buff = new AddressableLEDBuffer(LENGTH);

        actions.add(new ShowSolidLights(() -> drivebase.getLastGoodState() == Drivebase.State.ALIGN, Color.kGreen));
        actions.add(new ShowAllianceSolidLights(() -> drivebase.getLastGoodState() == Drivebase.State.FIELD_RELATIVE));
        actions.add(new BlinkAlternateColors(() -> drivebase.getLastGoodState() == Drivebase.State.BALANCE, Color.kGreen, Color.kBlack, 3));
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        leds.start();
    }

    @Override
    public final void update(final TorqueMode mode) {
        for (final LightAction action : actions) {
            if (action.condition.getAsBoolean()) {
                action.run(buff);
                leds.setData(buff);
                break;
            }
        }
    }

    private static abstract class LightAction {
        public final BooleanSupplier condition; 

        public LightAction(final BooleanSupplier condition) {
            this.condition = condition;
        }

        public abstract void run(AddressableLEDBuffer buff);
    }

    public static class ShowSolidLights extends LightAction {
        private final Color color;
    
        public ShowSolidLights(final BooleanSupplier condition, final Color color) {
            super(condition);
            this.color = color;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (int i = 0; i < LENGTH; i++)
                buff.setLED(i, color);
        }
    }

    public static class ShowAllianceSolidLights extends LightAction {
        public ShowAllianceSolidLights(final BooleanSupplier condition) {
            super(condition);
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            final Color color = DriverStation.getAlliance() == Alliance.Blue ? Color.kBlue : Color.kRed;

            for (int i = 0; i < LENGTH; i++)
                buff.setLED(i, color);
        }
    }

    public static class BlinkAlternateColors extends LightAction {
        private final Color color1, color2;
        private final double hertz;
    
        public BlinkAlternateColors(final BooleanSupplier condition, final Color color1, final Color color2, final double hertz) {
            super(condition);
            this.color1 = color1;
            this.color2 = color2;
            this.hertz = hertz;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            final double timestamp = TorqueUtil.time();
            final boolean on = (Math.floor(timestamp * hertz) % 2 == 1);

            for (int i = 0; i < LENGTH; i++)
                buff.setLED(i, on ? color1 : color2);
        }
    }

    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }
}
