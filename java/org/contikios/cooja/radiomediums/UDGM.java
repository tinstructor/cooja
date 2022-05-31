/*
 * Copyright (c) 2009, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

package org.contikios.cooja.radiomediums;

import java.util.Collection;
import java.util.Observable;
import java.util.Observer;
import java.util.Random;

import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.LogManager;
import org.contikios.cooja.contikimote.interfaces.TwofacedRadio;
import org.jdom.Element;

import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Mote;
import org.contikios.cooja.RadioConnection;
import org.contikios.cooja.SimEventCentral.MoteCountListener;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.interfaces.Position;
import org.contikios.cooja.interfaces.Radio;
import org.contikios.cooja.plugins.Visualizer;
import org.contikios.cooja.plugins.skins.UDGMVisualizerSkin;

/**
 * The Unit Disk Graph Radio Medium abstracts radio transmission range as circles.
 * 
 * It uses two different range parameters: one for transmissions, and one for
 * interfering with other radios and transmissions.
 * 
 * Both radio ranges grow with the radio output power indicator.
 * The range parameters are multiplied with [output power]/[maximum output power].
 * For example, if the transmission range is 100m, the current power indicator 
 * is 50, and the maximum output power indicator is 100, then the resulting transmission 
 * range becomes 50m.
 * 
 * For radio transmissions within range, two different success ratios are used [0.0-1.0]:
 * one for successful transmissions, and one for successful receptions.
 * If the transmission fails, no radio will hear the transmission.
 * If one of receptions fail, only that receiving radio will not receive the transmission,
 * but will be interfered throughout the entire radio connection.  
 * 
 * The received radio packet signal strength grows inversely with the distance to the
 * transmitter.
 *
 * @see #SS_STRONG
 * @see #SS_WEAK
 * @see #SS_NOTHING
 *
 * @see DirectedGraphMedium
 * @see UDGMVisualizerSkin
 * @author Fredrik Osterlind
 * @author Robbe Elsas
 */
@ClassDescription("Unit Disk Graph Medium (UDGM): Distance Loss")
public class UDGM extends AbstractRadioMedium {
  private static final Logger logger = LogManager.getLogger(UDGM.class);

  public double SUCCESS_RATIO_TX = 1.0; /* Success ratio of TX. If this fails, no radios receive the packet */
  public double SUCCESS_RATIO_RX = 1.0; /* Success ratio of RX. If this fails, the single affected receiver does not receive the packet */
  public double TRANSMITTING_RANGE_2400 = 50; /* Transmission range. */
  public double TRANSMITTING_RANGE_868 = 70; /* Transmission range for secondary. */
  public double INTERFERENCE_RANGE_2400 = 100; /* Interference range. Ignored if below transmission range. */
  public double INTERFERENCE_RANGE_868 = 120; /* Interference range for secondary. Ignored if below transmission range. */

  private DirectedGraphMedium dgrm; /* Used only for efficient destination lookup */

  private Random random = null;

  public UDGM(Simulation simulation) {
    super(simulation);
    random = simulation.getRandomGenerator();
    dgrm = new DirectedGraphMedium() {
      @Override
      protected void analyzeEdges() {
        /* Create edges according to distances.
         * XXX May be slow for mobile networks */
        clearEdges();
        for (Radio source: UDGM.this.getRegisteredRadios()) {
          Position sourcePos = source.getPosition();
          for (Radio dest: UDGM.this.getRegisteredRadios()) {
            Position destPos = dest.getPosition();
            /* Ignore ourselves */
            /* We're checking the mote instead of the Radio because
             * a mote may possess multiple Radio interfaces. */
            if (source.getMote() == dest.getMote()) {
              logger.info("Not adding edge because both interfaces belong to mote with ID = " + source.getMote().getID());
              continue;
            }
            /* TODO check if the following is really necessary */
            /* Ignore dest interfaces of a different type */
            if (source.getClass() != dest.getClass()) {
              logger.info("Not adding edge because both interfaces are of different type");
              continue;
            }
            double distance = sourcePos.getDistanceTo(destPos);
            if (distance < (source.getClass() == TwofacedRadio.class ? Math.max(TRANSMITTING_RANGE_868, INTERFERENCE_RANGE_868)
                    : Math.max(TRANSMITTING_RANGE_2400, INTERFERENCE_RANGE_2400))) {
              /* Add potential destination */
              addEdge(
                  new DirectedGraphMedium.Edge(source, 
                      new DGRMDestinationRadio(dest)));
            }
          }
        }
        super.analyzeEdges();
      }
    };

    /* Register as position observer.
     * If any positions change, re-analyze potential receivers. */
    final Observer positionObserver = new Observer() {
      @Override
      public void update(Observable o, Object arg) {
        dgrm.requestEdgeAnalysis();
      }
    };
    /* Re-analyze potential receivers if radios are added/removed. */
    simulation.getEventCentral().addMoteCountListener(new MoteCountListener() {
      @Override
      public void moteWasAdded(Mote mote) {
        mote.getInterfaces().getPosition().addObserver(positionObserver);
        dgrm.requestEdgeAnalysis();
      }
      @Override
      public void moteWasRemoved(Mote mote) {
        mote.getInterfaces().getPosition().deleteObserver(positionObserver);
        dgrm.requestEdgeAnalysis();
      }
    });
    for (Mote mote: simulation.getMotes()) {
      mote.getInterfaces().getPosition().addObserver(positionObserver);
    }
    dgrm.requestEdgeAnalysis();

    /* Register visualizer skin */
    Visualizer.registerVisualizerSkin(UDGMVisualizerSkin.class);
  }

  @Override
  public void removed() {
  	super.removed();
  	
		Visualizer.unregisterVisualizerSkin(UDGMVisualizerSkin.class);
  }
  
  public void setTxRange(double r) {
    TRANSMITTING_RANGE_2400 = r;
    dgrm.requestEdgeAnalysis();
  }

  public void setInterferenceRange(double r) {
    INTERFERENCE_RANGE_2400 = r;
    dgrm.requestEdgeAnalysis();
  }

  @Override
  public RadioConnection createConnections(Radio sender) {
    RadioConnection newConnection = new RadioConnection(sender);

    /* Fail radio transmission randomly - no radios will hear this transmission */
    if (getTxSuccessProbability(sender) < 1.0 && random.nextDouble() > getTxSuccessProbability(sender)) {
      return newConnection;
    }

    /* Calculate ranges: grows with radio output power */
    double moteTransmissionRange = (sender.getClass() == TwofacedRadio.class ? TRANSMITTING_RANGE_868 : TRANSMITTING_RANGE_2400)
    * ((double) sender.getCurrentOutputPowerIndicator() / (double) sender.getOutputPowerIndicatorMax());
    double moteInterferenceRange = (sender.getClass() == TwofacedRadio.class ? INTERFERENCE_RANGE_868 : INTERFERENCE_RANGE_2400)
    * ((double) sender.getCurrentOutputPowerIndicator() / (double) sender.getOutputPowerIndicatorMax());

    /* Get all potential destination radios */
    DestinationRadio[] potentialDestinations = dgrm.getPotentialDestinations(sender);
    if (potentialDestinations == null) {
      return newConnection;
    }

    /* Loop through all potential destinations */
    Position senderPos = sender.getPosition();
    for (DestinationRadio dest: potentialDestinations) {
      Radio recv = dest.radio;

      /* TODO check if the following is really necessary */
      /* Ignore ourselves */
      /* We're checking the mote instead of the Radio because
       * a mote may possess multiple Radio interfaces. */
      if (sender.getMote() == recv.getMote()) {
        continue;
      }
      /* TODO check if the following is really necessary */
      /* Ignore dest interfaces of a different type */
      if (sender.getClass() != recv.getClass()) {
        continue;
      }

      /* Fail if radios are on different (but configured) channels */ 
      if (sender.getChannel() >= 0 &&
          recv.getChannel() >= 0 &&
          sender.getChannel() != recv.getChannel()) {

        /* Add the connection in a dormant state;
           it will be activated later when the radio will be
           turned on and switched to the right channel. This behavior
           is consistent with the case when receiver is turned off. */
        newConnection.addInterfered(recv);

        continue;
      }
      Position recvPos = recv.getPosition();

      /* Fail if radio is turned off */
//      if (!recv.isReceiverOn()) {
//        /* Special case: allow connection if source is Contiki radio, 
//         * and destination is something else (byte radio).
//         * Allows cross-level communication with power-saving MACs. */
//        if (sender instanceof ContikiRadio &&
//            !(recv instanceof ContikiRadio)) {
//          /*logger.info("Special case: creating connection to turned off radio");*/
//        } else {
//          recv.interfereAnyReception();
//          continue;
//        }
//      }

      double distance = senderPos.getDistanceTo(recvPos);
      if (distance <= moteTransmissionRange) {
        /* Within transmission range */

        if (!recv.isRadioOn()) {
          newConnection.addInterfered(recv);
          recv.interfereAnyReception();
        } else if (recv.isInterfered()) {
          /* Was interfered: keep interfering */
          newConnection.addInterfered(recv);
        } else if (recv.isTransmitting()) {
          newConnection.addInterfered(recv);
        } else if (recv.isReceiving() || recv.isReceivingCorrupt() ||
            (random.nextDouble() > getRxSuccessProbability(sender, recv))) {
          /* Was receiving, or reception failed: start interfering */
          newConnection.addInterfered(recv);
          recv.interfereAnyReception();

          /* Interfere receiver in all other active radio connections */
          for (RadioConnection conn : getActiveConnections()) {
            if (conn.isDestination(recv)) {
              conn.addInterfered(recv);
            }
          }

        } else {
          /* Success: radio starts receiving */
          newConnection.addDestination(recv);
        }
      } else if (distance <= moteInterferenceRange) {
        /* Within interference range */
        newConnection.addInterfered(recv);
        recv.interfereAnyReception();
      }
    }

    return newConnection;
  }
  
  public double getSuccessProbability(Radio source, Radio dest) {
  	return getTxSuccessProbability(source) * getRxSuccessProbability(source, dest);
  }
  public double getTxSuccessProbability(Radio source) {
    return SUCCESS_RATIO_TX;
  }
  public double getRxSuccessProbability(Radio source, Radio dest) {
  	double distance = source.getPosition().getDistanceTo(dest.getPosition());
    double distanceSquared = Math.pow(distance,2.0);
    double distanceMax = (source.getClass() == TwofacedRadio.class ? TRANSMITTING_RANGE_868 : TRANSMITTING_RANGE_2400)
    * ((double) source.getCurrentOutputPowerIndicator() / (double) source.getOutputPowerIndicatorMax());
    if (distanceMax == 0.0) {
      return 0.0;
    }
    double distanceMaxSquared = Math.pow(distanceMax,2.0);
    double ratio = distanceSquared / distanceMaxSquared;
    if (ratio > 1.0) {
    	return 0.0;
    }
    return 1.0 - ratio*(1.0-SUCCESS_RATIO_RX);
  }

  @Override
  public void updateSignalStrengths() {
    /* Override: uses distance as signal strength factor */
    
    /* Reset signal strengths */
    for (Radio radio : getRegisteredRadios()) {
      radio.setCurrentSignalStrength(getBaseRssi(radio));
    }

    /* Set signal strength to below strong on destinations */
    RadioConnection[] conns = getActiveConnections();
    for (RadioConnection conn : conns) {
      if (conn.getSource().getCurrentSignalStrength() < SS_STRONG) {
        conn.getSource().setCurrentSignalStrength(SS_STRONG);
      }
      for (Radio dstRadio : conn.getDestinations()) {
        if (conn.getSource().getChannel() >= 0 &&
            dstRadio.getChannel() >= 0 &&
            conn.getSource().getChannel() != dstRadio.getChannel()) {
          continue;
        }

        double dist = conn.getSource().getPosition().getDistanceTo(dstRadio.getPosition());

        double maxTxDist = (conn.getSource().getClass() == TwofacedRadio.class ? TRANSMITTING_RANGE_868 : TRANSMITTING_RANGE_2400)
        * ((double) conn.getSource().getCurrentOutputPowerIndicator() / (double) conn.getSource().getOutputPowerIndicatorMax());
        double distFactor = dist/maxTxDist;

        double signalStrength = SS_STRONG + distFactor*(SS_WEAK - SS_STRONG);
        if (dstRadio.getCurrentSignalStrength() < signalStrength) {
          dstRadio.setCurrentSignalStrength(signalStrength);
        }
      }
    }

    /* Set signal strength to below weak on interfered */
    for (RadioConnection conn : conns) {
      for (Radio intfRadio : conn.getInterfered()) {
        if (conn.getSource().getChannel() >= 0 &&
            intfRadio.getChannel() >= 0 &&
            conn.getSource().getChannel() != intfRadio.getChannel()) {
          continue;
        }

        double dist = conn.getSource().getPosition().getDistanceTo(intfRadio.getPosition());

        double maxTxDist = (conn.getSource().getClass() == TwofacedRadio.class ? TRANSMITTING_RANGE_868 : TRANSMITTING_RANGE_2400)
        * ((double) conn.getSource().getCurrentOutputPowerIndicator() / (double) conn.getSource().getOutputPowerIndicatorMax());
        double distFactor = dist/maxTxDist;

        if (distFactor < 1) {
          double signalStrength = SS_STRONG + distFactor*(SS_WEAK - SS_STRONG);
          if (intfRadio.getCurrentSignalStrength() < signalStrength) {
            intfRadio.setCurrentSignalStrength(signalStrength);
          }
        } else {
          intfRadio.setCurrentSignalStrength(SS_WEAK);
          if (intfRadio.getCurrentSignalStrength() < SS_WEAK) {
            intfRadio.setCurrentSignalStrength(SS_WEAK);
          }
        }

        if (!intfRadio.isInterfered()) {
          /*logger.warn("Radio was not interfered: " + intfRadio);*/
          intfRadio.interfereAnyReception();
        }
      }
    }
  }

  @Override
  public Collection<Element> getConfigXML() {
    Collection<Element> config = super.getConfigXML();
    Element element;

    /* Transmitting range for 2.4 Ghz channel */
    element = new Element("transmitting_range_2400");
    element.setText(Double.toString(TRANSMITTING_RANGE_2400));
    config.add(element);

    /* Transmitting range for 868 MHz channel */
    element = new Element("transmitting_range_868");
    element.setText(Double.toString(TRANSMITTING_RANGE_868));
    config.add(element);

    /* Interference range for 2.4 Ghz channel */
    element = new Element("interference_range_2400");
    element.setText(Double.toString(INTERFERENCE_RANGE_2400));
    config.add(element);

    /* Interference range for 868 MHz channel */
    element = new Element("interference_range_868");
    element.setText(Double.toString(INTERFERENCE_RANGE_868));
    config.add(element);

    /* Transmission success probability */
    element = new Element("success_ratio_tx");
    element.setText("" + SUCCESS_RATIO_TX);
    config.add(element);

    /* Reception success probability */
    element = new Element("success_ratio_rx");
    element.setText("" + SUCCESS_RATIO_RX);
    config.add(element);

    return config;
  }

  @Override
  public boolean setConfigXML(Collection<Element> configXML, boolean visAvailable) {
    super.setConfigXML(configXML, visAvailable);
    for (Element element : configXML) {
      if (element.getName().equals("transmitting_range_2400")) {
        TRANSMITTING_RANGE_2400 = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("transmitting_range_868")) {
        TRANSMITTING_RANGE_868 = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("interference_range_2400")) {
        INTERFERENCE_RANGE_2400 = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("interference_range_868")) {
        INTERFERENCE_RANGE_868 = Double.parseDouble(element.getText());
      }

      /* Backwards compatibility */
      if (element.getName().equals("success_ratio")) {
        SUCCESS_RATIO_TX = Double.parseDouble(element.getText());
        logger.warn("Loading old Cooja Config, XML element \"sucess_ratio\" parsed at \"sucess_ratio_tx\"");
      }

      if (element.getName().equals("success_ratio_tx")) {
        SUCCESS_RATIO_TX = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("success_ratio_rx")) {
        SUCCESS_RATIO_RX = Double.parseDouble(element.getText());
      }
    }
    return true;
  }

}
