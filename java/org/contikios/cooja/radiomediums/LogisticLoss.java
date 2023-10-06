/*
 * Copyright (c) 2018-2019, University of Bristol
 * Copyright (c) 2021-2023, Ghent University
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

import java.util.*;

import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.LogManager;
import org.contikios.cooja.contikimote.interfaces.ContikiRadio;
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
import org.contikios.cooja.plugins.skins.LogisticLossVisualizerSkin;

/**
 * The LogisticLoss radio medium aims to be more realistic than the UDGM radio medium while
 * remaining as easily usable.
 * <p>
 * It takes its name from the fact that the logistic function (shaped as a sigmoid) is used to model
 * the packet reception probability based on RSSI.
 * <p>
 * Features:<p> - Models a non-linear relationship between signal level packet reception
 * probability<p> - Models the signal level as a function of distance following a standard formula
 * from the RF propagation theory<p> - Adds a random level of noise (AWGN) to the signal level of
 * each packet (shadowing)<p> - Multiple Configurable parameters<p> - Visualization similar to UDGM
 * visualization
 * <p>
 * This Cooja plugin uses a logistic function to model the PRR-RSSI relationship:
 * <p>
 * PRR(rssi) =  1.0 / (1 + exp(-(rssi - rssi_50%))),
 * <p>
 * where:<p> - `rssi` is the transmit-power minus the path loss.<p> - `rssi_50%` is the signal level
 * at which 50% packets are received;
 * <p>
 * To model the path loss PL_{dBm}(d) this plugin uses the log-distance path loss model with
 * log-normal shadowing (see Rappaport, T.S. Wireless Communications, Principles and Practice)
 * <p>
 * PL_{dBm}(d) = PL_0 + PL_t + 10 * \alpha * \log_10 (d / d_0) + NormalDistribution(0, \sigma),
 * <p>
 * where:<p> - `d_0` a close-in reference distance in the transmitter's far-field region;<p> -
 * `PL_0` is the path loss at `d_0` (to be calculated with the free-space path loss model, which
 * itself is derived from the Friis transmission formula);<p> - `PL_t` is the time-varying component
 * of the path loss (by default, zero);<p> - `\alpha` is the path loss exponent;<p> - `\sigma` is
 * the standard deviation of the Additive White Gaussian Noise;<p> - `NormalDistribution(0, \sigma)`
 * is a Gaussian-distributed random variable with zero-mean and standard deviation `\sigma`;
 * <p>
 * The default value of `\alpha` (the path loss exponent) is 3.0 and the default value of `\sigma`
 * is 3.0 as well, both of which approximately correspond to "indoors, 2.4 GHz frequency" according
 * to RF propagation theory. For 868 MHz `\alpha` is also set to 3.0 while `\sigma` is set to 5.0 by
 * default.
 * <p>
 * If the time-varying behavior is enabled, the value of `PL_t` is changing over time. The change is
 * within bounds `[TVPL_{min}, TVPL_{max}]`. The evolution is done in discrete steps. At the time
 * `t`, the `PL_t` is updated as:
 * <p>
 * PL_t = bound(PL_{t-1} + r),
 * <p>
 * where `r` is a small random value, and `bound(pl) = min(MAX_PL, max(MIN_PL, pl))`, and `MIN_PL`
 * and `MAX_PL` are time minimum and maximum values of the time-varying path loss.
 *
 * @author Atis Elsts
 * @author Robbe Elsas
 * @see UDGM
 */
@ClassDescription("LogisticLoss Medium")
public class LogisticLoss extends AbstractRadioMedium {

  private static final Logger logger = LogManager.getLogger(LogisticLoss.class);

  private final Simulation sim;

  /**
   * Success ratio of TX. If this fails, no radios receive the packet
   */
  public double SUCCESS_RATIO_TX = 1.0;

  /**
   * signal strength in dBm with close to 0% PRR
   */
  public double RX_SENSITIVITY_DBM = -100.0;

  /**
   * This is the point where the second-order derivative of the logistic loss function becomes
   * negative. It is also the point where 50% of packets with this signal strength are received.
   */
  public double RSSI_INFLECTION_POINT_DBM = -92.0;

  /**
   * For the log-distance model, indoors, 2.4 GHz
   */
  public double PATH_LOSS_EXPONENT_2400 = 3.0;

  /**
   * This is required to implement the Capture Effect. The co-channel rejection threshold of
   * 802.15.4 radios typically is -3 dB.
   */
  private static final double CO_CHANNEL_REJECTION = -3.0;

  /**
   * The transmission power.
   * TODO figure out how to getCurrentOutputPowerIndicator() to dBm use that.
   */
  public static final double DEFAULT_TX_POWER_DBM = 0.0;

  /**
   * Gain of the antennas used, in dBi.
   */
  public final double ANTENNA_GAIN_DBI = 0.0;

  /**
   * Close-in reference distance in the far-field of the transmitter
   */
  public double REFERENCE_DISTANCE = 1.0;

  /**
   * At this distance (in meters), the RSSI is equal to the RX_SENSITIVITY_DBM
   * TODO remove and use method getMaxTxRange() in the visualizer instead
   */
  public double TRANSMITTING_RANGE_2400 = REFERENCE_DISTANCE * Math.pow(10.0,
      (DEFAULT_TX_POWER_DBM - RX_SENSITIVITY_DBM + 2.0 * ANTENNA_GAIN_DBI
          - 20.0 * Math.log10(4.0 * Math.PI * 2.4 * REFERENCE_DISTANCE / 0.3)) / (10.0
          * PATH_LOSS_EXPONENT_2400));
  public double INTERFERENCE_RANGE_2400 = TRANSMITTING_RANGE_2400;

  /**
   * Enable the time-varying component?
   */
  public boolean ENABLE_TIME_VARIATION = false;

  /**
   * Does a jammer only interfere with the same comm mode?
   */
  public boolean INTERFERE_SAME_MODE_ONLY = false;

  /**
   * Bounds for the time-varying component
   */
  public double TIME_VARIATION_MIN_PL_DB = -10;
  public double TIME_VARIATION_MAX_PL_DB = +10;

  /**
   * How often to update the time-varying path loss value (in simulation time)?
   */
  private static final double TIME_VARIATION_STEP_SEC = 10.0;

  private long lastTimeVariationUpdatePeriod = 0;

  /**
   * Used only for efficient destination lookup
   */
  private final DirectedGraphMedium dgrm;

  private final Random random;

  private final HashMap<Index, TimeVaryingEdge> edgesTable = new HashMap<>();

  private static final ObservableMap<Class<? extends Radio>, Map<Integer, Double>> awgnSigmaMap = new ObservableMap<>();

  private static final ObservableMap<Class<? extends Radio>, Map<Integer, Double>> pathLossExponentMap = new ObservableMap<>();

  static {
    /* Define AWGN sigma values for ContikiRadio class */
    Map<Integer, Double> radioAWGNSigmas = new HashMap<>();
    radioAWGNSigmas.put(1, 3.0);
    radioAWGNSigmas.put(2, 3.0);

    /* Define AWGN sigma values for TwofacedRadio class */
    Map<Integer, Double> twofacedAWGNSigmas = new HashMap<>();
    twofacedAWGNSigmas.put(1, 5.0);
    twofacedAWGNSigmas.put(2, 5.0);

    /* Define path loss exponent values for Radio class */
    Map<Integer, Double> radioPathLossExponents = new HashMap<>();
    radioPathLossExponents.put(1, 3.0);
    radioPathLossExponents.put(2, 2.9);

    /* Define path loss exponent values for TwofacedRadio class */
    Map<Integer, Double> twofacedPathLossExponents = new HashMap<>();
    twofacedPathLossExponents.put(1, 3.0);
    twofacedPathLossExponents.put(2, 3.1);

    awgnSigmaMap.put(ContikiRadio.class, radioAWGNSigmas);
    awgnSigmaMap.put(TwofacedRadio.class, twofacedAWGNSigmas);

    pathLossExponentMap.put(ContikiRadio.class, radioPathLossExponents);
    pathLossExponentMap.put(TwofacedRadio.class, twofacedPathLossExponents);
  }

  public LogisticLoss(Simulation simulation) {
    super(simulation);
    random = simulation.getRandomGenerator();
    sim = simulation;
    dgrm = new DirectedGraphMedium() {
      @Override
      protected void analyzeEdges() {
        /* Create edges according to distances.
           May be slow for mobile networks */
        clearEdges();
        /* Do not remove the time-varying edges to preserve their evolution */

        for (Radio source : LogisticLoss.this.getRegisteredRadios()) {
          Position sourcePos = source.getPosition();
          int sourceID = source.getMote().getID();
          for (Radio dest : LogisticLoss.this.getRegisteredRadios()) {
            Position destPos = dest.getPosition();
            /* Ignore ourselves */
            /* We're checking the mote instead of the Radio because
               a mote may possess multiple Radio interfaces. */
            if (source.getMote() == dest.getMote()) {
              continue;
            }
            /* TODO check if the following is really necessary */
            /* Ignore dest interfaces of a different type */
            if (source.getClass() != dest.getClass()) {
              continue;
            }
            double distance = sourcePos.getDistanceTo(destPos);
            /* We need to check for the max range amongst all modes of of the given radio
               interface */
            double maxTxRange = getMaxTxRange(source);
            if (maxTxRange > 0.0 && distance <= maxTxRange) {
              /* Add potential destination */
              addEdge(new DirectedGraphMedium.Edge(source, new DGRMDestinationRadio(dest)));

              if (ENABLE_TIME_VARIATION) {
                int destID = dest.getMote().getID();
                if (sourceID < destID) {
                  Index key = new Index(sourceID, destID);
                  if (!edgesTable.containsKey(key)) {
                    edgesTable.put(key, new TimeVaryingEdge());
                  }
                }
              }
            }
          }
        }
        super.analyzeEdges();
      }
    };

    /* Register as position observer.
       If any positions change, re-analyze potential receivers. */
    final Observer positionObserver = (o, arg) -> dgrm.requestEdgeAnalysis();
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
    for (Mote mote : simulation.getMotes()) {
      mote.getInterfaces().getPosition().addObserver(positionObserver);
    }
    /* Register as observer of awgnSigmaMap and pathLossExponentMap */
    awgnSigmaMap.addObserver((o, arg) -> dgrm.requestEdgeAnalysis());
    pathLossExponentMap.addObserver((o, arg) -> dgrm.requestEdgeAnalysis());
    dgrm.requestEdgeAnalysis();

    /* Register visualizer skin */
    Visualizer.registerVisualizerSkin(LogisticLossVisualizerSkin.class);
  }

  @Override
  public void removed() {
    super.removed();

    Visualizer.unregisterVisualizerSkin(LogisticLossVisualizerSkin.class);
  }

  @Override
  public RadioConnection createConnections(Radio sender) {
    RadioConnection newConnection = new RadioConnection(sender);

    /* Fail radio transmission randomly - no radios will hear this transmission */
    if (getTxSuccessProbability() < 1.0 && random.nextDouble() > getTxSuccessProbability()) {
      return newConnection;
    }

    /* Get all potential destination radios */
    DestinationRadio[] potentialDestinations = dgrm.getPotentialDestinations(sender);
    if (potentialDestinations == null) {
      return newConnection;
    }

    /* Loop through all potential destinations */
    Position senderPos = sender.getPosition();
    for (DestinationRadio dest : potentialDestinations) {
      Radio recv = dest.radio;

      /* TODO check if the following is really necessary */
      /* Ignore ourselves */
      /* We're checking the mote instead of the Radio because
         a mote may possess multiple Radio interfaces. */
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
      /* If the send sends corrupt frames (meaning it is a jammer) we
         might (for some reason) want the (corrupt) packets it sends
         to only impact transmissions that use the same communication
         mode (this might be useful for testing) */
      if (INTERFERE_SAME_MODE_ONLY &&
          sender.sendsCorruptFrames() &&
          sender.getCommMode() != recv.getCommMode()) {
        /* Add the connection in a dormant state */
        newConnection.addInterfered(recv);
        continue;
      }

      Position recvPos = recv.getPosition();
      double distance = senderPos.getDistanceTo(recvPos);
      /* Using getTxRange(sender) is appropriate here because mode may have changed */
      double txRange = getTxRange(sender);
      if (txRange > 0.0 && distance <= txRange) {
        /* Within transmission range */

        if (!recv.isRadioOn()) {
          newConnection.addInterfered(recv);
          recv.interfereAnyReception();
        } else if (recv.isInterfered()) {
          /* Was interfered: keep interfering */
          newConnection.addInterfered(recv);
        } else if (recv.isTransmitting()) {
          newConnection.addInterfered(recv);
        } else {
          boolean receiveNewOk = random.nextDouble() < getRxSuccessProbability(sender, recv);

          if (recv.isReceiving() || recv.isReceivingCorrupt()) {
            /* Compare new and old and decide whether to interfere.
               This is a simplified check. Rather than looking at all N potential senders,
               it looks at just this and the strongest one of the previous transmissions
               (since updateSignalStrengths() updates the signal strength iff the previous one
               is weaker) */
            double oldSignal = recv.getCurrentSignalStrength();
            double newSignal = getRSSI(sender, recv);
            boolean doInterfereOld;

            if (oldSignal + CO_CHANNEL_REJECTION > newSignal) {
              /* keep the old transmission */
              doInterfereOld = false;
              receiveNewOk = false;
              /* logger.info(sender + ": keep old " + recv); */
            } else if (newSignal + CO_CHANNEL_REJECTION > oldSignal) {
              /* keep the new transmission */
              doInterfereOld = true;
              /* logger.info(sender + ": keep new " + recv); */
            } else {
              /* too equal strengths; none gets through */
              doInterfereOld = true;
              receiveNewOk = false;

              /* logger.info(sender + ": interfere both " + recv); */

              /* This will interfere even if later a stronger connections
                 comes ahead that could override all existing weaker connections! */
              recv.interfereAnyReception();
            }

            if (doInterfereOld) {
              /* Find all existing connections and interfere them */
              for (RadioConnection conn : getActiveConnections()) {
                if (conn.isDestination(recv)) {
                  conn.addInterfered(recv);
                }
              }

              recv.interfereAnyReception();
            }
          }

          /* If new transmission uses different mode, it can't be ok */
          if (sender.getCommMode() >= 0 &&
              recv.getCommMode() >= 0 &&
              sender.getCommMode() != recv.getCommMode()) {
            receiveNewOk = false;
          }

          if (receiveNewOk) {
            /* Success: radio starts receiving */
            newConnection.addDestination(recv);
            /* logger.info(sender + ": tx to " + recv); */
          } else {
            newConnection.addInterfered(recv);
            /* logger.info(sender + ": interfere to " + recv); */
          }
        }
      }
    }

    return newConnection;
  }

  public double getSuccessProbability(Radio source, Radio dest) {
    return getTxSuccessProbability() * getRxSuccessProbability(source, dest);
  }

  public double getTxSuccessProbability() {
    return SUCCESS_RATIO_TX;
  }

  public double getRxSuccessProbability(Radio source, Radio dest) {
    double rssi = getRSSI(source, dest);
    double x = rssi - RSSI_INFLECTION_POINT_DBM;
    return 1.0 / (1.0 + Math.exp(-x));
  }

  private double getCenterFrequencyInGHz(Radio radio) {
    return radio.getClass() == TwofacedRadio.class ? 0.868 : 2.4;
  }

  private double getPathLossIndBm(Radio source, double distance) {
    double pathLossExponent = getPathLossExponent(source);
    if (pathLossExponent > 0) {
      return getPathLossAtReferenceDistance(source) + 10 * pathLossExponent * Math.log10(
          (distance > 0 ? distance : 0.01) / REFERENCE_DISTANCE);
    }
    return 0.0;
  }

  /**
   * The path loss at a close-in reference distance in the far-field region of the transmitter. It
   * is common practive to calculate it from the free-space path loss equation:
   * <p>
   * PL_0 = 20 * \log_10 (4 * PI * f * d_0 / c) - AG_tx - AG_rx
   * <p>
   * where:<p> - `d_0` is the close-in reference distance in m;<p> - `PL_0` is the loss at `d_0` in
   * dBm;<p> - `AG_tx` is the transmitter antenna gain in dBi;<p> - `AG_rx` is the receiver antenna
   * gain in dBi;<p> - `f` is the operating frequency in Hz;<p> - `c` is the speed of light = 300
   * 000 000 m/s.
   *
   * @param source Source radio that is exactly 1 reference distance away
   * @return Path loss (in dBm) at reference distance associated with this medium
   * @see REFERENCE_DISTANCE
   */
  private double getPathLossAtReferenceDistance(Radio source) {
    return 20 * Math.log10(4 * Math.PI * getCenterFrequencyInGHz(source) * REFERENCE_DISTANCE / 0.3)
        - 2 * ANTENNA_GAIN_DBI;
  }

  private double getAWGNSigma(Radio source) {
    Class<? extends Radio> radioType = source.getClass();
    if (awgnSigmaMap.containsKey(radioType)) {
      Map<Integer, Double> awgnSigmas = awgnSigmaMap.get(radioType);
      int commMode = source.getCommMode();
      if (awgnSigmas.containsKey(commMode)) {
        return awgnSigmas.get(commMode);
      }
    }
    return 0.0;
  }

  private double getPathLossExponent(Radio source) {
    Class<? extends Radio> radioType = source.getClass();
    if (pathLossExponentMap.containsKey(radioType)) {
      Map<Integer, Double> pathLossExponents = pathLossExponentMap.get(radioType);
      int commMode = source.getCommMode();
      if (pathLossExponents.containsKey(commMode)) {
        return pathLossExponents.get(commMode);
      }
    }
    return 0.0;
  }

  /**
   * At this distance (in meters), the RSSI is equal to the RX_SENSITIVITY_DBM. More specifically,
   * given:
   * <p>
   * |PL_max| = PL_tx - rssi = 20 * \log_10 (4 * PI * f * d_0 / c) - AG_tx - AG_rx + 10 * \alpha *
   * \log_10 (d_max / d_0)
   * <p>
   * where:<p> - `d_0` is the close-in reference distance in meters;<p> - `PL_0` is the loss at
   * `d_0` in dBm;<p> - `d_max` is the transmission range in meters;<p> - `PL_max` is the loss at
   * `d_max` in dBm;<p> - `PL_tx` is the transmit power in dBm;<p> - `\alpha` is the path loss
   * exponent;<p> - `AG_tx` is the transmitter antenna gain in dBi;<p> - `AG_rx` is the receiver
   * antenna gain in dBi;<p> - `f` is the operating frequency in Hz;<p> - `c` is the speed of light
   * = 300 000 000 m/s.
   * <p>
   * Then, if we isolate for d_max, we get:
   * <p>
   * d_max = d_ref * 10 ^ ((PL_tx - rssi + AG_tx + AG_rx - 20 * \log_10 (4 * PI * f * d_ref / c)) /
   * (\alpha * 10))
   * <p>
   * For example, assuming the rx sensitivity (i.e., the rx power below which the PRR is
   * approximately 0%) equals -100 dBm, assuming the rssi is a good approximation of the actual rx
   * power, and given f = 2.4 GHz, AG_tx = AG_rx = 0 dBi, \alpha = 3, PL_tx = 0 dBm, and d_ref = 1.0
   * m, we get:
   * <p>
   * d_max = 1.0 m * 10 ^ ((100 dBm - 20 * \log_10 (4 * PI * 2.4 GHz * 1.0 m / 0.3 Gm/s)) / (3 *
   * 10))<p> = 99.648 m
   *
   * @param source Source radio from which you'd receive a packet with an RSSI equal to
   *               RX_SENSITIVITY_DBM at the transmission range for the currently configured
   *               communication mode
   * @return Distance from source (in meters) at which RSSI equals RX_SENSITIVITY_DBM
   * @see RX_SENSITIVITY_DBM
   */
  private double getTxRange(Radio source) {
    double pathLossExponent = getPathLossExponent(source);
    if (pathLossExponent > 0) {
      return REFERENCE_DISTANCE * Math.pow(10.0,
          (DEFAULT_TX_POWER_DBM - RX_SENSITIVITY_DBM + 2.0 * ANTENNA_GAIN_DBI - 20.0 * Math.log10(
              4.0 * Math.PI * getCenterFrequencyInGHz(source) * REFERENCE_DISTANCE / 0.3)) / (10.0
              * pathLossExponent));
    }
    return 0.0;
  }

  private double getMaxTxRange(Radio source) {
    double maxTxRange = 0.0;
    Class<? extends Radio> radioType = source.getClass();
    if (pathLossExponentMap.containsKey(radioType)) {
      Map<Integer, Double> pathLossExponents = pathLossExponentMap.get(radioType);
      for (Double pathLossExponent : pathLossExponents.values()) {
        double txRange = REFERENCE_DISTANCE * Math.pow(10.0,
            (DEFAULT_TX_POWER_DBM - RX_SENSITIVITY_DBM + 2.0 * ANTENNA_GAIN_DBI - 20.0 * Math.log10(
                4.0 * Math.PI * getCenterFrequencyInGHz(source) * REFERENCE_DISTANCE / 0.3)) / (10.0
                * pathLossExponent));
        maxTxRange = txRange > maxTxRange ? txRange : maxTxRange;
      }
    }
    return maxTxRange;
  }

  private double getInterferenceRange(Radio source) {
    return getTxRange(source);
  }

  /* Additive White Gaussian Noise, sampled from the distribution N(0.0, AWGN_SIGMA) */
  private double getAWGN(Radio source) {
    return random.nextGaussian() * getAWGNSigma(source);
  }

  private double getRSSI(Radio source, Radio dst) {
    /* Using the log-distance formula */
    double path_loss_dbm = getPathLossIndBm(source,
        source.getPosition().getDistanceTo(dst.getPosition()));

    /* Add the time-varying component if enabled */
    if (ENABLE_TIME_VARIATION) {
      Index key = new Index(source.getMote().getID(), dst.getMote().getID());
      TimeVaryingEdge e = edgesTable.get(key);
      if (e != null) {
        path_loss_dbm += e.getPL();
      } else {
        logger.warn(
            "No edge between " + source.getMote().getID() + " and " + dst.getMote().getID());
      }
    }

    /* getAWGN() may cause an exception to be thrown when reloading a simulation */
    /* The solution is to simply not call it when the simulation is not running! */
    return DEFAULT_TX_POWER_DBM - path_loss_dbm + (sim.isRunning() ? getAWGN(source) : 0);
  }

  private void updateTimeVariationComponent() {
    long period = (long) (sim.getSimulationTimeMillis() / (1000.0 * TIME_VARIATION_STEP_SEC));

    if (dgrm.needsEdgeAnalysis()) {
      dgrm.analyzeEdges();
    }

    while (period > lastTimeVariationUpdatePeriod) {
      for (Map.Entry<Index, TimeVaryingEdge> entry : edgesTable.entrySet()) {
        entry.getValue().evolve();
      }
      /* update the time state */
      lastTimeVariationUpdatePeriod += 1;
    }
  }

  @Override
  public void updateSignalStrengths() {
    /* Override: uses distance as signal strength factor */

    if (ENABLE_TIME_VARIATION) {
      updateTimeVariationComponent();
    }

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
        if (INTERFERE_SAME_MODE_ONLY &&
            conn.getSource().sendsCorruptFrames() &&
            conn.getSource().getCommMode() != dstRadio.getCommMode()) {
          continue;
        }

        double rssi = getRSSI(conn.getSource(), dstRadio);
        if (dstRadio.getCurrentSignalStrength() < rssi) {
          dstRadio.setCurrentSignalStrength(rssi);
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
        if (INTERFERE_SAME_MODE_ONLY &&
            conn.getSource().sendsCorruptFrames() &&
            conn.getSource().getCommMode() != intfRadio.getCommMode()) {
          continue;
        }

        double rssi = getRSSI(conn.getSource(), intfRadio);
        if (intfRadio.getCurrentSignalStrength() < rssi) {
          intfRadio.setCurrentSignalStrength(rssi);
        }

        /* NOTE uncomment the following if there is a desire to see broken packets false
            wakeups in all cases, not just in the case of collision, as happens at the
            moment */
/*
        if (!intfRadio.isInterfered()) {
          logger.warn("Radio was not interfered: " + intfRadio);
          intfRadio.interfereAnyReception();
        }
*/
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

    /* Transmission success probability */
    element = new Element("success_ratio_tx");
    element.setText(String.valueOf(SUCCESS_RATIO_TX));
    config.add(element);

    /* Close-in reference distance */
    element = new Element("reference_distance");
    element.setText(String.valueOf(REFERENCE_DISTANCE));
    config.add(element);

    /* Rx sensitivity */
    element = new Element("rx_sensitivity");
    element.setText(String.valueOf(RX_SENSITIVITY_DBM));
    config.add(element);

    /* RSSI inflection point */
    element = new Element("rssi_inflection_point");
    element.setText(String.valueOf(RSSI_INFLECTION_POINT_DBM));
    config.add(element);

    /* Path loss exponent for 2.4 Ghz channel */
    element = new Element("path_loss_exponent_2400");
    element.setText(String.valueOf(PATH_LOSS_EXPONENT_2400));
    config.add(element);

    /* Time variation enabled? */
    element = new Element("enable_time_variation");
    element.setText(String.valueOf(ENABLE_TIME_VARIATION));
    config.add(element);

    if (ENABLE_TIME_VARIATION) {
      /* Time-variable path loss bounds */
      element = new Element("time_variation_min_pl_db");
      element.setText(String.valueOf(TIME_VARIATION_MIN_PL_DB));
      config.add(element);
      element = new Element("time_variation_max_pl_db");
      element.setText(String.valueOf(TIME_VARIATION_MAX_PL_DB));
      config.add(element);
    }

    /* Does a jammer only interfere with the same comm mode? */
    element = new Element("interfere_same_mode_only");
    element.setText(String.valueOf(INTERFERE_SAME_MODE_ONLY));
    config.add(element);

    /* Serialize AWGN sigma map */
    Element awgnSigmaMapElement = new Element("awgn_sigma_map");
    for (Class<? extends Radio> radioType : awgnSigmaMap.keySet()) {
      Map<Integer, Double> awgnSigmas = awgnSigmaMap.get(radioType);
      Element radioTypeElement = new Element("radio_type");
      radioTypeElement.setAttribute("class", radioType.getName());
      for (Integer commMode : awgnSigmas.keySet()) {
        Element sigmaElement = new Element("sigma");
        sigmaElement.setAttribute("comm_mode", commMode.toString());
        sigmaElement.setText(awgnSigmas.get(commMode).toString());
        radioTypeElement.addContent(sigmaElement);
      }
      awgnSigmaMapElement.addContent(radioTypeElement);
    }
    config.add(awgnSigmaMapElement);

    /* Serialize path loss exponent map */
    Element pathLossExponentMapElement = new Element("path_loss_exponent_map");
    for (Class<? extends Radio> radioType : pathLossExponentMap.keySet()) {
      Map<Integer, Double> pathLossExponents = pathLossExponentMap.get(radioType);
      Element radioTypeElement = new Element("radio_type");
      radioTypeElement.setAttribute("class", radioType.getName());
      for (Integer commMode : pathLossExponents.keySet()) {
        Element exponentElement = new Element("exponent");
        exponentElement.setAttribute("comm_mode", commMode.toString());
        exponentElement.setText(pathLossExponents.get(commMode).toString());
        radioTypeElement.addContent(exponentElement);
      }
      pathLossExponentMapElement.addContent(radioTypeElement);
    }
    config.add(pathLossExponentMapElement);

    return config;
  }

  @Override
  public boolean setConfigXML(Collection<Element> configXML, boolean visAvailable) {
    super.setConfigXML(configXML, visAvailable);
    for (Element element : configXML) {
      if (element.getName().equals("transmitting_range_2400")) {
        TRANSMITTING_RANGE_2400 = Double.parseDouble(element.getText());
        INTERFERENCE_RANGE_2400 = TRANSMITTING_RANGE_2400;
      }

      if (element.getName().equals("success_ratio_tx")) {
        SUCCESS_RATIO_TX = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("reference_distance")) {
        REFERENCE_DISTANCE = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("rx_sensitivity")) {
        RX_SENSITIVITY_DBM = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("rssi_inflection_point")) {
        RSSI_INFLECTION_POINT_DBM = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("path_loss_exponent_2400")) {
        PATH_LOSS_EXPONENT_2400 = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("enable_time_variation")) {
        ENABLE_TIME_VARIATION = Boolean.parseBoolean(element.getText());
      }

      if (element.getName().equals("interfere_same_mode_only")) {
        INTERFERE_SAME_MODE_ONLY = Boolean.parseBoolean(element.getText());
      }

      if (element.getName().equals("time_variation_min_pl_db")) {
        TIME_VARIATION_MIN_PL_DB = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("time_variation_max_pl_db")) {
        TIME_VARIATION_MAX_PL_DB = Double.parseDouble(element.getText());
      }

      if (element.getName().equals("awgn_sigma_map")) {
        /* Deserialize AWGN sigma map */
        List<Element> radioTypeElements = element.getChildren("radio_type");
        for (Element radioTypeElement : radioTypeElements) {
          String radioClassName = radioTypeElement.getAttributeValue("class");
          Class<? extends Radio> radioType;
          try {
            radioType = (Class<? extends Radio>) Class.forName(radioClassName);
          } catch (ClassNotFoundException e) {
            /* Handle the exception if the class is not found */
            e.printStackTrace();
            continue;
          }
          Map<Integer, Double> awgnSigmas = new HashMap<>();
          List<Element> sigmaElements = radioTypeElement.getChildren("sigma");
          for (Element sigmaElement : sigmaElements) {
            int commMode = Integer.parseInt(sigmaElement.getAttributeValue("comm_mode"));
            double sigma = Double.parseDouble(sigmaElement.getText());
            awgnSigmas.put(commMode, sigma);
          }
          awgnSigmaMap.put(radioType, awgnSigmas);
        }
      }

      if (element.getName().equals("path_loss_exponent_map")) {
        /* Deserialize path loss exponent map */
        List<Element> radioTypeElements = element.getChildren("radio_type");
        for (Element radioTypeElement : radioTypeElements) {
          String radioClassName = radioTypeElement.getAttributeValue("class");
          Class<? extends Radio> radioType;
          try {
            radioType = (Class<? extends Radio>) Class.forName(radioClassName);
          } catch (ClassNotFoundException e) {
            /* Handle the exception if the class is not found */
            e.printStackTrace();
            continue;
          }
          Map<Integer, Double> pathLossExponents = new HashMap<>();
          List<Element> exponentElements = radioTypeElement.getChildren("exponent");
          for (Element exponentElement : exponentElements) {
            int commMode = Integer.parseInt(exponentElement.getAttributeValue("comm_mode"));
            double exponent = Double.parseDouble(exponentElement.getText());
            pathLossExponents.put(commMode, exponent);
          }
          pathLossExponentMap.put(radioType, pathLossExponents);
        }
      }

    }
    return true;
  }

  /**
   * Method to update a single AWGN sigma value
   *
   * @param radioClassName Name of radio class
   * @param mode           Mode id
   * @param sigma          AWGN sigma
   * @return true if successful, false otherwise
   */
  public boolean updateAWGNSigma(String radioClassName, int mode, double sigma) {
    Class<? extends Radio> radioClass = getRadioClassByName(radioClassName);
    if (radioClass != null) {
      Map<Integer, Double> sigmaMap = awgnSigmaMap.get(radioClass);
      if (sigmaMap != null && sigmaMap.containsKey(mode)) {
        sigmaMap.put(mode, sigma);
        awgnSigmaMap.put(radioClass, sigmaMap);
        return true;
      }
    }
    return false;
  }

  /**
   * Method to update a single path loss exponent value
   *
   * @param radioClassName Name of radio class
   * @param mode           Mode id
   * @param exponent       Path loss exponent
   * @return true if successful, false otherwise
   */
  public boolean updatePathLossExponent(String radioClassName, int mode, double exponent) {
    Class<? extends Radio> radioClass = getRadioClassByName(radioClassName);
    if (radioClass != null) {
      Map<Integer, Double> exponentMap = pathLossExponentMap.get(radioClass);
      if (exponentMap != null && exponentMap.containsKey(mode)) {
        exponentMap.put(mode, exponent);
        pathLossExponentMap.put(radioClass, exponentMap);
        return true;
      }
    }
    return false;
  }

  /**
   * Utility method to get a Class object from its name
   *
   * @param className Name of class
   * @return Class object if exists, null otherwise
   */
  private Class<? extends Radio> getRadioClassByName(String className) {
    try {
      return (Class<? extends Radio>) Class.forName(className);
    } catch (ClassNotFoundException e) {
      e.printStackTrace();
      return null;
    }
  }

  private static class ObservableMap<K, V> extends Observable {

    private Map<K, V> map;

    public ObservableMap() {
      this.map = new HashMap<>();
    }

    public void put(K key, V value) {
      map.put(key, value);
      setChanged();
      notifyObservers();
    }

    public V get(K key) {
      return map.get(key);
    }

    public Set<K> keySet() {
      return map.keySet();
    }

    public boolean containsKey(K key) {
      return map.containsKey(key);
    }
  }

  private static class Index {

    private final int x;
    private final int y;

    public Index(int a, int b) {
      if (a <= b) {
        this.x = a;
        this.y = b;
      } else {
        this.x = b;
        this.y = a;
      }
    }

    @Override
    public int hashCode() {
      return this.x ^ this.y;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (obj == null) {
        return false;
      }
      if (getClass() != obj.getClass()) {
        return false;
      }
      Index other = (Index) obj;
      if (x != other.x) {
        return false;
      }
      return y == other.y;
    }
  }

  private class TimeVaryingEdge {

    /* The current value of the time-varying */
    private double timeVariationPlDb;

    public TimeVaryingEdge() {
      timeVariationPlDb = 0.0;
    }

    public void evolve() {
      /* evolve the value */
      timeVariationPlDb += random.nextDouble() - 0.5;
      /* bound the value */
      if (timeVariationPlDb < TIME_VARIATION_MIN_PL_DB) {
        timeVariationPlDb = TIME_VARIATION_MIN_PL_DB;
      } else if (timeVariationPlDb > TIME_VARIATION_MAX_PL_DB) {
        timeVariationPlDb = TIME_VARIATION_MAX_PL_DB;
      }
    }

    public double getPL() {
      return timeVariationPlDb;
    }
  }
}
