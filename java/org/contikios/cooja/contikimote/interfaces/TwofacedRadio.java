/*
 * Copyright (c) 2021, Ghent University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.contikios.cooja.contikimote.interfaces;

import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.LogManager;
import org.contikios.cooja.*;
import org.contikios.cooja.contikimote.ContikiMote;
import org.contikios.cooja.interfaces.PolledAfterActiveTicks;
import org.contikios.cooja.interfaces.Position;
import org.contikios.cooja.interfaces.Radio;
import org.contikios.cooja.mote.memory.VarMemory;
import org.contikios.cooja.util.CCITT_CRC;
import org.jdom.Element;

import java.util.ArrayList;
import java.util.Collection;

/**
 * Secondary packet radio transceiver mote interface
 *
 * @author Robbe Elsas
 * @see ContikiRadio
 */
@ClassDescription("Twofaced Radio Interface")
public class TwofacedRadio extends Radio implements PolledAfterActiveTicks {
    private final ContikiMote mote;

    private final VarMemory myMoteMemory;

    private static final Logger logger = LogManager.getLogger(TwofacedRadio.class);

    private final CCITT_CRC txCrc = new CCITT_CRC();

    private RadioPacket packetToMote = null;

    private RadioPacket packetFromMote = null;

    private boolean radioOn;

    private boolean isTransmitting = false;

    private boolean isReceivingCorrupt = false;

    private boolean isInterfered = false;

    private long transmissionEndTime = -1;

    private RadioEvent lastEvent = RadioEvent.UNKNOWN;

    private long lastEventTime = 0;

    private int oldOutputPowerIndicator = -1;

    private int oldOutputPower = -1;

    private int oldRadioChannel = -1;

    private int oldCommMode = -1;

    private double oldTxRate = -1.0;

    private byte[] oldBrokenLink = {0, 0, 0, 0};

    private byte[] oldBrokenLink2 = {0, 0, 0, 0};

    public TwofacedRadio(Mote mote) {
        this.mote = (ContikiMote) mote;
        this.myMoteMemory = new VarMemory(mote.getMemory());

        radioOn = myMoteMemory.getByteValueOf("simRadioHWOnTwofaced") == 1;
    }

    @Override
    public Collection<Element> getConfigXML() {
        return null;
    }

    @Override
    public void setConfigXML(Collection<Element> configXML, boolean visAvailable) {
    }

    @Override
    public void doActionsAfterTick() {
        long now = mote.getSimulation().getSimulationTime();

        /* Check if radio hardware status changed */
        if (radioOn != (myMoteMemory.getByteValueOf("simRadioHWOnTwofaced") == 1)) {
            radioOn = !radioOn;

            if (!radioOn) {
                myMoteMemory.setByteValueOf("simReceivingTwofaced", (byte) 0);
                myMoteMemory.setIntValueOf("simInSizeTwofaced", 0);
                myMoteMemory.setIntValueOf("simOutSizeTwofaced", 0);
                isTransmitting = false;
                lastEvent = RadioEvent.HW_OFF;
            } else {
                lastEvent = RadioEvent.HW_ON;
            }

            lastEventTime = now;
            this.setChanged();
            this.notifyObservers();
        }
        if (!radioOn) {
            return;
        }

        /* Check if radio output power changed */
        if (myMoteMemory.getByteValueOf("simPowerTwofaced") != oldOutputPowerIndicator) {
            oldOutputPowerIndicator = myMoteMemory.getByteValueOf("simPowerTwofaced");
            lastEvent = RadioEvent.UNKNOWN;
            this.setChanged();
            this.notifyObservers();
        }

        if (getCurrentOutputPower() != oldOutputPower) {
            oldOutputPower = myMoteMemory.getIntValueOf("simPowerdBmTwofaced");
            lastEvent = RadioEvent.UNKNOWN;
            this.setChanged();
            this.notifyObservers();
        }

        /* Check if radio channel changed */
        if (getChannel() != oldRadioChannel) {
            oldRadioChannel = getChannel();
            lastEvent = RadioEvent.UNKNOWN;
            this.setChanged();
            this.notifyObservers();
        }

        /* Check if communication mode changed */
        if(getCommMode() != oldCommMode) {
            oldCommMode = getCommMode();
            lastEvent = RadioEvent.UNKNOWN;
            this.setChanged();
            this.notifyObservers();
        }

        /* Check if tx rate has changed */
        if(getTxRate() != oldTxRate) {
            oldTxRate = getTxRate();
            lastEvent = RadioEvent.UNKNOWN;
            this.setChanged();
            this.notifyObservers();
        }

        /* Check if broken link has changed */
        if(getBrokenLink() != oldBrokenLink) {
            oldBrokenLink = getBrokenLink();
            lastEvent = RadioEvent.UNKNOWN;
            this.setChanged();
            this.notifyObservers();
        }

        if(getBrokenLink2() != oldBrokenLink2) {
            oldBrokenLink2 = getBrokenLink2();
            lastEvent = RadioEvent.UNKNOWN;
            this.setChanged();
            this.notifyObservers();
        }

        /* Ongoing transmission */
        if (isTransmitting && now >= transmissionEndTime) {
            myMoteMemory.setIntValueOf("simOutSizeTwofaced", 0);
            isTransmitting = false;
            mote.requestImmediateWakeup();

            lastEventTime = now;
            lastEvent = RadioEvent.TRANSMISSION_FINISHED;
            this.setChanged();
            this.notifyObservers();
            /*logger.debug("----- CONTIKI TRANSMISSION ENDED -----");*/
        }

        /* New transmission */
        int size = myMoteMemory.getIntValueOf("simOutSizeTwofaced");
        if (!isTransmitting && size > 0) {
            packetFromMote = new COOJARadioPacket(myMoteMemory.getByteArray("simOutDataBufferTwofaced", size + 2));

            if (packetFromMote.getPacketData() == null || packetFromMote.getPacketData().length == 0) {
                logger.warn("Skipping zero sized Contiki packet (no buffer)");
                myMoteMemory.setIntValueOf("simOutSizeTwofaced", 0);
                mote.requestImmediateWakeup();
                return;
            }

            byte[] data = packetFromMote.getPacketData();
            txCrc.setCRC(0);
            for (int i = 0; i < size; i++) {
                txCrc.addBitrev(data[i]);
            }
            data[size] = (byte) txCrc.getCRCHi();
            data[size + 1] = (byte) txCrc.getCRCLow();

            isTransmitting = true;

            /* Calculate transmission duration (us) */
            /* XXX Currently floored due to millisecond scheduling! */
            long duration = (int) (Simulation.MILLISECOND * ((8 * size /*bits*/) / getTxRate()));
            transmissionEndTime = now + Math.max(1, duration);

            lastEventTime = now;
            lastEvent = RadioEvent.TRANSMISSION_STARTED;
            this.setChanged();
            this.notifyObservers();
            //logger.debug("----- NEW CONTIKI TRANSMISSION DETECTED -----");

            // Deliver packet right away
            lastEvent = RadioEvent.PACKET_TRANSMITTED;
            this.setChanged();
            this.notifyObservers();
            //logger.debug("----- CONTIKI PACKET DELIVERED -----");
        }

        if (isTransmitting && transmissionEndTime > now) {
            mote.scheduleNextWakeup(transmissionEndTime);
        }
    }

    @Override
    public void setReceivedPacket(RadioPacket packet) {
        packetToMote = packet;
    }

    @Override
    public RadioPacket getLastPacketTransmitted() {
        return packetFromMote;
    }

    @Override
    public RadioPacket getLastPacketReceived() {
        return packetToMote;
    }

    @Override
    public void signalReceptionStart(Radio sender) {
        packetToMote = null;
        /*
         * Drop the incoming transmission if this radio is currently: being interfered
         * (as determined by the radio medium), already receiving a different transmission,
         * or currently transmitting a signal itself. We don't check for isReceivingCorrupt()
         * here because if an incoming corrupt message did not cause the medium to set this
         * radio as interfered, then it can't cause the new transmission to be dropped.
         * Note that it seems redundant to check both isInterfered() and isReceiving(), but
         * you must remember that not all mediums have an equal transmitting and interference
         * range.
         */
        if (isInterfered() || isReceiving() || isTransmitting()) {
            interfereAnyReception();
            return;
        }

        if (sender.sendsCorruptFrames()) {
            isReceivingCorrupt = true;
        } else {
            myMoteMemory.setByteValueOf("simReceivingTwofaced", (byte) 1);
            mote.requestImmediateWakeup();
        }

        lastEventTime = mote.getSimulation().getSimulationTime();
        lastEvent = RadioEvent.RECEPTION_STARTED;

        if (!sender.sendsCorruptFrames()) {
            myMoteMemory.setInt64ValueOf("simLastPacketTimestampTwofaced", lastEventTime);
        }

        this.setChanged();
        this.notifyObservers();
    }

    @Override
    public void signalReceptionEnd(Radio sender) {
        if (isInterfered || packetToMote == null) {
            isInterfered = false;
            packetToMote = null;
            if (!sender.sendsCorruptFrames()) {
                myMoteMemory.setIntValueOf("simInSizeTwofaced", 0);
            }
        } else if (!sender.sendsCorruptFrames()){
            myMoteMemory.setIntValueOf("simInSizeTwofaced", packetToMote.getPacketData().length - 2);
            myMoteMemory.setByteArray("simInDataBufferTwofaced", packetToMote.getPacketData());
        }

        if (sender.sendsCorruptFrames()) {
            isReceivingCorrupt = false;
        } else {
            myMoteMemory.setByteValueOf("simReceivingTwofaced", (byte) 0);
            mote.requestImmediateWakeup();
        }
        lastEventTime = mote.getSimulation().getSimulationTime();
        lastEvent = RadioEvent.RECEPTION_FINISHED;
        this.setChanged();
        this.notifyObservers();
    }

    @Override
    public RadioEvent getLastEvent() {
        return lastEvent;
    }

    @Override
    public boolean isTransmitting() {
        return isTransmitting;
    }

    @Override
    public boolean isReceiving() {
        return myMoteMemory.getByteValueOf("simReceivingTwofaced") == 1;
    }

    @Override
    public boolean isInterfered() {
        return isInterfered;
    }

    @Override
    public boolean isRadioOn() {
        return radioOn;
    }

    @Override
    public void interfereAnyReception() {
        if (isInterfered()) {
            return;
        }
        isInterfered = true;
        lastEvent = RadioEvent.RECEPTION_INTERFERED;
        lastEventTime = mote.getSimulation().getSimulationTime();
        this.setChanged();
        this.notifyObservers();
    }

    @Override
    public double getCurrentOutputPower() {
        return myMoteMemory.getIntValueOf("simPowerdBmTwofaced");
    }

    @Override
    public int getCurrentOutputPowerIndicator() {
        return myMoteMemory.getByteValueOf("simPowerTwofaced");
    }

    @Override
    public int getOutputPowerIndicatorMax() {
        return 100;
    }

    @Override
    public double getCurrentSignalStrength() {
        return myMoteMemory.getIntValueOf("simSignalStrengthTwofaced");
    }

    @Override
    public void setCurrentSignalStrength(double signalStrength) {
        myMoteMemory.setIntValueOf("simSignalStrengthTwofaced", (int) signalStrength);
    }

    @Override
    public int getChannel() {
        return myMoteMemory.getIntValueOf("simRadioChannelTwofaced");
    }

    @Override
    public int getCommMode() {
        return myMoteMemory.getIntValueOf("simCommModeTwofaced");
    }

    @Override
    public double getTxRate() {
        return myMoteMemory.getIntValueOf("simTxRateTwofaced");
    }

    @Override
    public byte[] getBrokenLink() {
        return myMoteMemory.getByteArray("simBrokenLinkTwofaced", 4);
    }

    @Override
    public byte[] getBrokenLink2() {
        return myMoteMemory.getByteArray("simBrokenLinkTwofaced2", 4);
    }

    @Override
    public Position getPosition() {
        return mote.getInterfaces().getPosition();
    }

    @Override
    public Mote getMote() {
        return mote;
    }

    @Override
    public boolean sendsCorruptFrames() {
        return myMoteMemory.getByteValueOf("simCorruptFrames") == 1;
    }

    @Override
    public boolean isReceivingCorrupt() {
        return isReceivingCorrupt;
    }

    @Override
    public int getLQI() throws UnsupportedOperationException {
        return myMoteMemory.getIntValueOf("simLQITwofaced");
    }

    @Override
    public void setLQI(int lqi) {
        lqi = Math.min(Math.max(lqi, 0), 0xff);
        myMoteMemory.setIntValueOf("simLQI", lqi);
    }

    @Override
    public String toString() {
        return "TwofacedRadio at " + mote;
    }
}
