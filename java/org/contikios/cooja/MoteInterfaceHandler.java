/*
 * Copyright (c) 2008, Swedish Institute of Computer Science.
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

package org.contikios.cooja;

import static java.util.Map.entry;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.contikios.cooja.contikimote.ContikiMoteType;
import org.contikios.cooja.contikimote.interfaces.ContikiBeeper;
import org.contikios.cooja.contikimote.interfaces.ContikiButton;
import org.contikios.cooja.contikimote.interfaces.ContikiCFS;
import org.contikios.cooja.contikimote.interfaces.ContikiClock;
import org.contikios.cooja.contikimote.interfaces.ContikiEEPROM;
import org.contikios.cooja.contikimote.interfaces.ContikiLED;
import org.contikios.cooja.contikimote.interfaces.ContikiMoteID;
import org.contikios.cooja.contikimote.interfaces.ContikiPIR;
import org.contikios.cooja.contikimote.interfaces.ContikiRS232;
import org.contikios.cooja.contikimote.interfaces.ContikiRadio;
import org.contikios.cooja.contikimote.interfaces.ContikiVib;
import org.contikios.cooja.contikimote.interfaces.TwofacedRadio;
import org.contikios.cooja.interfaces.Battery;
import org.contikios.cooja.interfaces.Beeper;
import org.contikios.cooja.interfaces.Button;
import org.contikios.cooja.interfaces.Clock;
import org.contikios.cooja.interfaces.IPAddress;
import org.contikios.cooja.interfaces.LED;
import org.contikios.cooja.interfaces.Log;
import org.contikios.cooja.interfaces.Mote2MoteRelations;
import org.contikios.cooja.interfaces.MoteAttributes;
import org.contikios.cooja.interfaces.MoteID;
import org.contikios.cooja.interfaces.PIR;
import org.contikios.cooja.interfaces.Position;
import org.contikios.cooja.interfaces.Radio;
import org.contikios.cooja.interfaces.RimeAddress;
import org.contikios.cooja.motes.DisturberMoteType;
import org.contikios.cooja.motes.ImportAppMoteType;
import org.contikios.cooja.mspmote.SkyMoteType;
import org.contikios.cooja.mspmote.Z1MoteType;
import org.contikios.cooja.mspmote.interfaces.Msp802154Radio;
import org.contikios.cooja.mspmote.interfaces.MspClock;
import org.contikios.cooja.mspmote.interfaces.MspDebugOutput;
import org.contikios.cooja.mspmote.interfaces.MspDefaultSerial;
import org.contikios.cooja.mspmote.interfaces.MspLED;
import org.contikios.cooja.mspmote.interfaces.MspMoteID;
import org.contikios.cooja.mspmote.interfaces.MspSerial;
import org.contikios.cooja.mspmote.interfaces.SkyCoffeeFilesystem;
import org.contikios.cooja.mspmote.interfaces.SkyFlash;
import org.contikios.cooja.mspmote.interfaces.SkyTemperature;
import org.contikios.cooja.radiomediums.DirectedGraphMedium;
import org.contikios.cooja.radiomediums.LogisticLoss;
import org.contikios.cooja.radiomediums.SilentRadioMedium;
import org.contikios.cooja.radiomediums.UDGM;
import org.contikios.cooja.radiomediums.UDGMConstantLoss;
import org.contikios.mrm.MRM;

/**
 * The mote interface handler holds all interfaces for a specific mote.
 *
 * @author Fredrik Osterlind
 * @author Robbe Elsas
 */
public class MoteInterfaceHandler {
  private static final Logger logger = LogManager.getLogger(MoteInterfaceHandler.class);

  /** Static translation map from name -> class for builtin interfaces. */
  private static final Map<String, Class<? extends MoteInterface>> builtinInterfaces = Map.ofEntries(
          entry("org.contikios.cooja.interfaces.IPAddress", IPAddress.class),
          entry("org.contikios.cooja.interfaces.Position", Position.class),
          entry("org.contikios.cooja.interfaces.Battery", Battery.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiVib", ContikiVib.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiMoteID", ContikiMoteID.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiRS232", ContikiRS232.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiBeeper", ContikiBeeper.class),
          entry("org.contikios.cooja.interfaces.RimeAddress", RimeAddress.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiIPAddress", IPAddress.class), // Compatibility.
          entry("org.contikios.cooja.contikimote.interfaces.ContikiRadio", ContikiRadio.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiButton", ContikiButton.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiPIR", ContikiPIR.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiClock", ContikiClock.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiLED", ContikiLED.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiCFS", ContikiCFS.class),
          entry("org.contikios.cooja.contikimote.interfaces.ContikiEEPROM", ContikiEEPROM.class),
          entry("org.contikios.cooja.interfaces.Mote2MoteRelations", Mote2MoteRelations.class),
          entry("org.contikios.cooja.interfaces.MoteAttributes", MoteAttributes.class),
          entry("org.contikios.cooja.mspmote.interfaces.ESBLog", MspSerial.class), // Compatibility.
          entry("org.contikios.cooja.mspmote.interfaces.MspClock", MspClock.class),
          entry("org.contikios.cooja.mspmote.interfaces.MspDebugOutput", MspDebugOutput.class),
          entry("org.contikios.cooja.mspmote.interfaces.MspDefaultSerial", MspDefaultSerial.class),
          entry("org.contikios.cooja.mspmote.interfaces.MspIPAddress", IPAddress.class), // Compatibility.
          entry("org.contikios.cooja.mspmote.interfaces.MspLED", MspLED.class),
          entry("org.contikios.cooja.mspmote.interfaces.MspMoteID", MspMoteID.class),
          entry("org.contikios.cooja.mspmote.interfaces.Msp802154Radio", Msp802154Radio.class),
          entry("org.contikios.cooja.mspmote.interfaces.SkyByteRadio", Msp802154Radio.class), // Compatibility.
          entry("org.contikios.cooja.mspmote.interfaces.SkyCoffeeFilesystem", SkyCoffeeFilesystem.class),
          entry("org.contikios.cooja.mspmote.interfaces.SkyFlash", SkyFlash.class),
          entry("org.contikios.cooja.mspmote.interfaces.SkySerial", MspSerial.class), // Compatibility.
          entry("org.contikios.cooja.mspmote.interfaces.SkyTemperature", SkyTemperature.class));

  private final ArrayList<MoteInterface> moteInterfaces = new ArrayList<>();

  /* Cached interfaces */
  private Battery myBattery;
  private Beeper myBeeper;
  private Button myButton;
  private Clock myClock;
  private IPAddress myIPAddress;
  private RimeAddress myRimeAddress;
  private LED myLED;
  private Log myLog;
  private MoteID myMoteID;
  private PIR myPIR;
  private Position myPosition;
  private Radio myRadio;
  private Radio myTwofacedRadio;

  /**
   * Creates new mote interface handler. All given interfaces are created.
   *
   * @param mote Mote
   * @param interfaceClasses Mote interface classes
   */
  public MoteInterfaceHandler(Mote mote, Class<? extends MoteInterface>[] interfaceClasses) throws MoteType.MoteTypeCreationException {
    for (Class<? extends MoteInterface> interfaceClass : interfaceClasses) {
      try {
        moteInterfaces.add(interfaceClass.getConstructor(Mote.class).newInstance(mote));
      } catch (Exception e) {
        logger.fatal("Exception when calling constructor of " + interfaceClass, e);
        throw new MoteType.MoteTypeCreationException("Exception when calling constructor of " + interfaceClass, e);
      }
    }
  }

  /** Fast translation from class name to object for builtin mote types.
   * @param cooja Cooja
   * @param name Name of mote type to create
   * @return Object or null
   */
  public static MoteType createMoteType(Cooja cooja, String name) {
    return switch (name) {
      case "org.contikios.cooja.motes.ImportAppMoteType" -> new ImportAppMoteType();
      case "org.contikios.cooja.motes.DisturberMoteType" -> new DisturberMoteType();
      case "org.contikios.cooja.contikimote.ContikiMoteType" -> new ContikiMoteType(cooja);
      case "org.contikios.cooja.mspmote.SkyMoteType" -> new SkyMoteType();
      case "org.contikios.cooja.mspmote.Z1MoteType" -> new Z1MoteType();
      default -> null;
    };
  }

  /** Fast translation from class name to object for radio mediums.
   * @param sim Simulation
   * @param name Name of radio medium to create
   * @return Object or null
   */
  public static RadioMedium createRadioMedium(Simulation sim, String name) {
    if (name.startsWith("se.sics")) {
      name = name.replaceFirst("se\\.sics", "org.contikios");
    }
    switch (name) {
      case "org.contikios.cooja.radiomediums.UDGM": return new UDGM(sim);
      case "org.contikios.cooja.radiomediums.UDGMConstantLoss": return new UDGMConstantLoss(sim);
      case "org.contikios.cooja.radiomediums.DirectedGraphMedium": return new DirectedGraphMedium(sim);
      case "org.contikios.cooja.radiomediums.SilentRadioMedium": return new SilentRadioMedium(sim);
      case "org.contikios.cooja.radiomediums.LogisticLoss": return new LogisticLoss(sim);
      case "org.contikios.cooja.mrm.MRM": return new MRM(sim);
    }
    var clazz = sim.getCooja().tryLoadClass(sim, RadioMedium.class, name);
    if (clazz == null) {
      return null;
    }
    try {
      return clazz.getConstructor(Simulation.class).newInstance(sim);
    } catch (Exception e) {
      return null;
    }
  }

  /** Fast translation from class name to class file for builtin interfaces. Uses the classloader
   * to load other interfaces.
   * @param gui Cooja
   * @param caller Object calling
   * @param name Name of class to find
   * @return Found class or null
   */
  public static Class<? extends MoteInterface> getInterfaceClass(Cooja gui, Object caller, String name) {
    if (name.startsWith("se.sics")) {
      name = name.replaceFirst("se\\.sics", "org.contikios");
    }
    var clazz = builtinInterfaces.get(name);
    if (clazz != null) {
      return clazz;
    }
    return gui.tryLoadClass(caller, MoteInterface.class, name);
  }

  /**
   * Returns interface of given type. Returns the first interface found that
   * is either of the given class or of a subclass (except {@link TwofacedRadio},
   * unless explicitly specified).
   * <p>
   *
   * Usage: getInterfaceOfType(Radio.class)
   *
   * @param <N>
   * @param interfaceType Class of interface to return
   * @return Mote interface, or null if no interface exists of given type
   */
  public <N extends MoteInterface> N getInterfaceOfType(Class<N> interfaceType) {
    for (MoteInterface intf : moteInterfaces) {
      if (interfaceType.isInstance(intf) && (!(intf instanceof TwofacedRadio) ||
              (interfaceType == TwofacedRadio.class))) {
        return interfaceType.cast(intf);
      }
    }

    return null;
  }

  /**
   * Returns the first interface with a class name that ends with the given arguments.
   * Example: mote.getInterfaces().get("Temperature");
   * 
   * @param classname
   * @return
   */
  public MoteInterface get(String classname) {
    for (MoteInterface intf : moteInterfaces) {
      if (intf.getClass().getName().endsWith(classname)) {
        return intf;
      }
    }
    return null;
  }

  /**
   * Returns the battery interface (if any).
   *
   * @return Battery interface
   */
  public Battery getBattery() {
    if (myBattery == null) {
      myBattery = getInterfaceOfType(Battery.class);
    }
    return myBattery;
  }

  /**
   * Returns the beeper interface (if any).
   *
   * @return Beeper interface
   */
  public Beeper getBeeper() {
    if (myBeeper == null) {
      myBeeper = getInterfaceOfType(Beeper.class);
    }
    return myBeeper;
  }

  /**
   * Returns the button interface (if any).
   *
   * @return Button interface
   */
  public Button getButton() {
    if (myButton == null) {
      myButton = getInterfaceOfType(Button.class);
    }
    return myButton;
  }

  /**
   * Returns the clock interface (if any).
   *
   * @return Clock interface
   */
  public Clock getClock() {
    if (myClock == null) {
      myClock = getInterfaceOfType(Clock.class);
    }
    return myClock;
  }

  /**
   * Returns the IP address interface (if any).
   *
   * @return IP Address interface
   */
  public IPAddress getIPAddress() {
    if (myIPAddress == null) {
      myIPAddress = getInterfaceOfType(IPAddress.class);
    }
    return myIPAddress;
  }

  /**
   * @return Rime address interface
   */
  public RimeAddress getRimeAddress() {
    if (myRimeAddress == null) {
      myRimeAddress = getInterfaceOfType(RimeAddress.class);
    }
    return myRimeAddress;
  }

  /**
   * Returns the LED interface (if any).
   *
   * @return LED interface
   */
  public LED getLED() {
    if (myLED == null) {
      myLED = getInterfaceOfType(LED.class);
    }
    return myLED;
  }

  /**
   * Returns the log interface (if any).
   *
   * @return Log interface
   */
  public Log getLog() {
    if (myLog == null) {
      myLog = getInterfaceOfType(Log.class);
    }
    return myLog;
  }

  /**
   * Returns the mote ID interface (if any).
   *
   * @return Mote ID interface
   */
  public MoteID getMoteID() {
    if (myMoteID == null) {
      myMoteID = getInterfaceOfType(MoteID.class);
    }
    return myMoteID;
  }

  /**
   * Returns the PIR interface (if any).
   *
   * @return PIR interface
   */
  public PIR getPIR() {
    if (myPIR == null) {
      myPIR = getInterfaceOfType(PIR.class);
    }
    return myPIR;
  }

  /**
   * Returns the position interface (if any).
   *
   * @return Position interface
   */
  public Position getPosition() {
    if (myPosition == null) {
      myPosition = getInterfaceOfType(Position.class);
    }
    return myPosition;
  }

  /**
   * Returns the radio interface (if any), except if it's an
   * instance of {@link TwofacedRadio}.
   *
   * @return Radio interface
   */
  public Radio getRadio() {
    if (myRadio == null) {
      myRadio = getInterfaceOfType(Radio.class);
    }
    return myRadio;
  }

  /**
   * Returns the twofaced radio interface (if any).
   *
   * @return Twofaced radio interface
   */
  public Radio getTwofacedRadio() {
    if (myTwofacedRadio == null) {
      myTwofacedRadio = getInterfaceOfType(TwofacedRadio.class);
    }
    return myTwofacedRadio;
  }

  /**
   * @return Mote interfaces
   */
  public Collection<MoteInterface> getInterfaces() {
    return moteInterfaces;
  }

  @Override
  public String toString() {
    return "Mote interfaces handler (" + moteInterfaces.size() + " mote interfaces)";
  }
}
