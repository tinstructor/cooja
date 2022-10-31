/*
 * Copyright (c) 2009, Swedish Institute of Computer Science. All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer. 2. Redistributions in
 * binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution. 3. Neither the name of the
 * Institute nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

package org.contikios.cooja;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Observable;
import java.util.Random;
import java.util.concurrent.LinkedBlockingDeque;
import javax.swing.JOptionPane;

import javax.swing.JTextArea;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.LogManager;
import org.jdom.Element;

import org.contikios.cooja.dialogs.CreateSimDialog;

/**
 * A simulation consists of a number of motes and mote types.
 * <p>
 * A simulation is observable:
 * changed simulation state, added or deleted motes etc. are observed.
 * To track mote changes, observe the mote (interfaces) itself.
 *
 * @author Fredrik Osterlind
 */
public class Simulation extends Observable {

  /** Commands sent to the simulation thread to start, stop, or shutdown the simulation */
  private enum Command {
    START, STOP, QUIT
  }

  public static final long MICROSECOND = 1L;
  public static final long MILLISECOND = 1000*MICROSECOND;

  /** Lock used to wait for simulation state changes */
  private final Object stateLock = new Object();

  private final ArrayList<Mote> motes = new ArrayList<>();
  private final ArrayList<MoteType> moteTypes = new ArrayList<>();

  private final LinkedBlockingDeque<Object> commandQueue = new LinkedBlockingDeque<>();

  private final Thread simulationThread;

  /* If true, run simulation at full speed */
  private boolean speedLimitNone = true;
  /* Limit simulation speed to maxSpeed; if maxSpeed is 1.0 simulation is run at real-time speed */
  private double speedLimit;
  /* Used to restrict simulation speed */
  private long speedLimitLastSimtime;
  private long speedLimitLastRealtime;

  private long lastStartRealTime;
  private long lastStartSimulationTime;
  private long currentSimulationTime = 0;

  private String title = null;

  private RadioMedium currentRadioMedium = null;

  private static final Logger logger = LogManager.getLogger(Simulation.class);

  private volatile boolean isRunning = false;
  private volatile boolean isShutdown = false;

  private final Cooja cooja;

  private long randomSeed;

  private boolean randomSeedGenerated = false;

  private long maxMoteStartupDelay = 1000*MILLISECOND;

  private final SafeRandom randomGenerator;

  /* Event queue */
  private final EventQueue eventQueue = new EventQueue();

  /** List of active script engines. */
  private final ArrayList<LogScriptEngine> scriptEngines = new ArrayList<>();

  /** The return value from startSimulation. */
  private volatile Integer returnValue = null;

  private final TimeEvent delayEvent = new TimeEvent() {
    @Override
    public void execute(long t) {
      if (speedLimitNone) {
        /* As fast as possible: no need to reschedule delay event */
        return;
      }

      long diffSimtime = getSimulationTimeMillis() - speedLimitLastSimtime; /* ms */
      long diffRealtime = System.currentTimeMillis() - speedLimitLastRealtime; /* ms */
      long expectedDiffRealtime = (long) (diffSimtime/speedLimit);
      long sleep = expectedDiffRealtime - diffRealtime;
      if (sleep >= 0) {
        scheduleEvent(this, t+MILLISECOND);
        /* Slow down simulation */
        try {
          Thread.sleep(sleep);
        } catch (InterruptedException e) {
          // Restore interrupted status
          Thread.currentThread().interrupt();
        }
      } else {
        /* Reduce slow-down: execute this delay event less often */
        scheduleEvent(this, t-sleep*MILLISECOND);
      }

      /* Update counters every second */
      if (diffRealtime > 1000) {
        speedLimitLastRealtime = System.currentTimeMillis();
        speedLimitLastSimtime = getSimulationTimeMillis();
      }
    }
    @Override
    public String toString() {
      return "DELAY";
    }
  };

  /**
   * Creates a new simulation
   */
  public Simulation(Cooja cooja, long seed) {
    this.cooja = cooja;
    randomGenerator = new SafeRandom(this);
    randomSeed = seed;
    simulationThread = new Thread(() -> {
      boolean isAlive = true;
      do {
        boolean isSimulationRunning = false;
        EventQueue.Pair nextEvent = null;
        try {
          while (isAlive) {
            Object cmd;
            do {
              cmd = isSimulationRunning ? commandQueue.poll() : commandQueue.take();
              if (cmd instanceof Runnable r) {
                r.run();
              } else if (cmd instanceof Command c) {
                isAlive = c != Command.QUIT;
                isShutdown = !isAlive;
                isSimulationRunning = c == Command.START;
                setRunning(isSimulationRunning);
              }
            } while (cmd != null && isAlive);

            if (isSimulationRunning) {
              /* Handle one simulation event, and update simulation time */
              nextEvent = eventQueue.popFirst();
              assert nextEvent != null : "Ran out of events in eventQueue";
              assert nextEvent.time >= currentSimulationTime : "Event from the past";
              currentSimulationTime = nextEvent.time;
              nextEvent.event.execute(currentSimulationTime);
            }
          }
        } catch (RuntimeException e) {
          if ("MSPSim requested simulation stop".equals(e.getMessage())) {
            logger.info("Simulation stopped due to MSPSim breakpoint");
          } else {
            logger.fatal("Simulation stopped due to error: " + e.getMessage(), e);
            if (!Cooja.isVisualized()) {
              /* Quit simulator if in test mode */
              System.exit(1);
            }
            String title = "Simulation error";
            if (nextEvent != null && nextEvent.event instanceof MoteTimeEvent moteTimeEvent) {
              title += ": " + moteTimeEvent.getMote();
            }
            Cooja.showErrorDialog(title, e, false);
          }
        } catch (InterruptedException e) {
          // Simulation thread interrupted - quit
          logger.warn("simulation thread interrupted");
          Thread.currentThread().interrupt();
          isAlive = false;
          isShutdown = true;
        }
        setRunning(false);
      } while (isAlive);
      isShutdown = true;
      commandQueue.clear();

      // Deactivate all script engines
      for (var engine : scriptEngines) {
        engine.deactivateScript();
        engine.closeLog();
      }

      // Remove the radio medium
      if (currentRadioMedium != null) {
        currentRadioMedium.removed();
      }

      // Remove all motes
      Mote[] motes = getMotes();
      for (Mote m: motes) {
        doRemoveMote(m);
      }

      // Log test status to console in headless mode, ScriptRunner shows status in GUI mode.
      if (!Cooja.isVisualized()) {
        if (returnValue == null) {
          logger.info("TEST OK\n");
        } else {
          logger.warn("TEST FAILED\n");
        }
      }
    }, "sim");
    simulationThread.start();
  }

  /**
   * Set if the simulation is running or not.
   * May only be called by the simulation thread to notify about its state.
   */
  private void setRunning(boolean isRunning) {
    if (this.isRunning == isRunning) {
      return;
    }

    if (isRunning) {
      // Simulation starting
      speedLimitLastRealtime = lastStartRealTime = System.currentTimeMillis();
      speedLimitLastSimtime = lastStartSimulationTime = getSimulationTimeMillis();
    } else {
      // Simulation stopped
      var realTimeDuration = System.currentTimeMillis() - lastStartRealTime;
      var simulationDuration = getSimulationTimeMillis() - lastStartSimulationTime;
      logger.info("Runtime: {} ms. Simulated time: {} ms. Speedup: {}",
                  realTimeDuration, simulationDuration,
                  ((double) simulationDuration / Math.max(1, realTimeDuration)));
    }

    synchronized (stateLock) {
      this.isRunning = isRunning;
      stateLock.notifyAll();
    }

    cooja.updateProgress(!isRunning);
    setChanged();
    notifyObservers(this);
  }

  /**
   * Request poll from simulation thread.
   * Poll requests are prioritized over simulation events, and are
   * executed between each simulation event.
   *
   * @param r Simulation thread action
   */
  public void invokeSimulationThread(Runnable r) {
    if (!isShutdown) {
      commandQueue.add(r);
    }
  }

  /**
   * @return True iff current thread is the simulation thread
   */
  public boolean isSimulationThread() {
    return simulationThread == Thread.currentThread();
  }

  /**
   * Schedule simulation event for given time.
   * Already scheduled events must be removed before they are rescheduled.
   * <p>
   * If the simulation is running, this method may only be called from the simulation thread.
   *
   * @see #invokeSimulationThread(Runnable)
   *
   * @param e Event
   * @param time Execution time
   */
  public void scheduleEvent(final TimeEvent e, final long time) {
    assert isSimulationThread() : "Scheduling event from non-simulation thread: " + e;
    eventQueue.addEvent(e, time);
  }

  /** Basic simulation configuration. */
  public record SimConfig(String title, String radioMedium, boolean generatedSeed, long randomSeed, long moteStartDelay) {}

  public SimConfig getSimConfig() {
    return new Simulation.SimConfig(title,
            currentRadioMedium == null ? null : Cooja.getDescriptionOf(currentRadioMedium),
            randomSeedGenerated, randomSeed, maxMoteStartupDelay / MILLISECOND);
  }

  public boolean setSimConfig(SimConfig cfg) throws MoteType.MoteTypeCreationException {
    if (cfg == null) {
      return false;
    }
    title = cfg.title;
    var radioMedium = MoteInterfaceHandler.createRadioMedium(this, cfg.radioMedium);
    if (radioMedium == null) {
      throw new MoteType.MoteTypeCreationException("Could not load " + cfg.radioMedium);
    }
    setRadioMedium(radioMedium);
    randomSeedGenerated = cfg.generatedSeed;
    setRandomSeed(cfg.randomSeed);
    maxMoteStartupDelay = Math.max(0, cfg.moteStartDelay);
    return true;
  }

  /** Create a new script engine that logs to the logTextArea and add it to the list
   *  of active script engines. */
  public LogScriptEngine newScriptEngine(JTextArea logTextArea) {
    var engine = new LogScriptEngine(this, scriptEngines.size(), logTextArea);
    scriptEngines.add(engine);
    return engine;
  }

  /** Remove a script engine from the list of active script engines. */
  public void removeScriptEngine(LogScriptEngine engine) {
    engine.deactivateScript();
    scriptEngines.remove(engine);
  }

  /**
   * Starts this simulation (notifies observers).
   */
  public void startSimulation() {
    startSimulation(false);
  }

  public Integer startSimulation(boolean block) {
    if (!isRunning() && !isShutdown) {
      commandQueue.add(Command.START);
      if (block) {
        try {
          // Wait for simulation to be shutdown
          simulationThread.join();
        } catch (InterruptedException e) {
          throw new RuntimeException(e);
        }
        return returnValue;
      }
    }
    return null;
  }

  /**
   * Stop simulation and block until it has stopped.
   */
  public void stopSimulation() {
    if (stopSimulation(null)) {
      waitFor(false, 250);
    }
  }

  /**
   * Stop simulation
   *
   * @param rv Return value from startSimulation, should be null unless > 0
   * @return True if action was taken
   */
  public boolean stopSimulation(Integer rv) {
    if (!isRunning() || isShutdown) {
      return false;
    }
    assert rv == null || rv > 0 : "Pass in rv = null or rv > 0";
    if (rv != null) {
      returnValue = rv;
    }
    commandQueue.add(Cooja.isVisualized() ? Command.STOP : Command.QUIT);
    return true;
  }

  private void waitFor(boolean isRunning, long timeout) {
    if (Thread.currentThread() == simulationThread) {
      return;
    }
    long startTime = System.currentTimeMillis();
    try {
      synchronized (stateLock) {
        while (this.isRunning != isRunning && !isShutdown) {
          long maxWaitTime = timeout - (System.currentTimeMillis() - startTime);
          if (timeout != 0 && maxWaitTime <= 0) {
            return;
          }
          stateLock.wait(timeout == 0 ? 0 : maxWaitTime);
        }
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  /**
   * Starts simulation if stopped, executes one millisecond, and finally stops
   * simulation again.
   */
  public void stepMillisecondSimulation() {
    if (isRunning()) {
      return;
    }
    TimeEvent stopEvent = new TimeEvent() {
      @Override
      public void execute(long t) {
        stopSimulation();
      }
    };
    scheduleEvent(stopEvent, getSimulationTime()+Simulation.MILLISECOND);
    startSimulation();
  }

  public Cooja getCooja() {
    return cooja;
  }

  /**
   * @return Random seed
   */
  public long getRandomSeed() {
    return randomSeed;
  }

  /**
   * @param randomSeed Random seed
   */
  public void setRandomSeed(long randomSeed) {
    this.randomSeed = randomSeed;
    randomGenerator.setSeed(randomSeed);
    String name =
      cooja.currentConfigFile == null ? "(unnamed)"
                                      : cooja.currentConfigFile.toString();
    logger.info("Simulation " + name + " random seed: " + randomSeed);
  }

  public Random getRandomGenerator() {
    return randomGenerator;
  }

  private final SimEventCentral eventCentral = new SimEventCentral(this);
  public SimEventCentral getEventCentral() {
    return eventCentral;
  }

  /**
   * Returns the current simulation config represented by XML elements. This
   * config also includes the current radio medium, all mote types and motes.
   *
   * @return Current simulation config
   */
  public Collection<Element> getConfigXML() {
    ArrayList<Element> config = new ArrayList<>();

    Element element;

    // Title
    element = new Element("title");
    element.setText(title);
    config.add(element);

    /* Max simulation speed */
    if (!speedLimitNone) {
      element = new Element("speedlimit");
      element.setText(String.valueOf(getSpeedLimit()));
      config.add(element);
    }

    // Random seed
    element = new Element("randomseed");
    if (randomSeedGenerated) {
      element.setText("generated");
    } else {
      element.setText(Long.toString(getRandomSeed()));
    }
    config.add(element);

    // Max mote startup delay
    element = new Element("motedelay_us");
    element.setText(Long.toString(maxMoteStartupDelay));
    config.add(element);

    // Radio Medium
    element = new Element("radiomedium");
    element.setText(currentRadioMedium.getClass().getName());

    Collection<Element> radioMediumXML = currentRadioMedium.getConfigXML();
    if (radioMediumXML != null) {
      element.addContent(radioMediumXML);
    }
    config.add(element);

    /* Event central */
    element = new Element("events");
    element.addContent(eventCentral.getConfigXML());
    config.add(element);

    // Mote types
    for (MoteType moteType : moteTypes) {
      element = new Element("motetype");
      element.setText(moteType.getClass().getName());

      Collection<Element> moteTypeXML = moteType.getConfigXML(this);
      if (moteTypeXML != null) {
        element.addContent(moteTypeXML);
      }
      config.add(element);
    }

    // Motes
    for (Mote mote : motes) {
      element = new Element("mote");

      Collection<Element> moteConfig = mote.getConfigXML();
      if (moteConfig == null) {
        moteConfig = new ArrayList<>();
      }

      /* Add mote type identifier */
      Element typeIdentifier = new Element("motetype_identifier");
      typeIdentifier.setText(mote.getType().getIdentifier());
      moteConfig.add(typeIdentifier);

      element.addContent(moteConfig);
      config.add(element);
    }

    return config;
  }

  
  /* indicator to components setting up that they need to respect the fast setup mode */
  private boolean quick = false;
  public boolean isQuickSetup() {
      return quick;
  }
  
  /**
   * Sets the current simulation config depending on the given configuration.
   *
   * @param root Simulation configuration
   * @return True if simulation was configured successfully
   * @throws Exception If configuration could not be loaded
   */
  public boolean setConfigXML(Element root, boolean quick) throws Exception {
    this.quick = quick;
    // Parse elements
    for (var element : (List<Element>) root.getChildren()) {
      switch (element.getName()) {
        case "title":
          title = element.getText();
          break;
        case "speedlimit": {
          String text = element.getText();
          if (text.equals("null")) {
            setSpeedLimit(null);
          } else {
            setSpeedLimit(Double.parseDouble(text));
          }
          break;
        }
        case "randomseed": {
          if (element.getText().equals("generated")) {
            randomSeedGenerated = true;
          }
          // Seed already passed into the constructor, init random generator.
          setRandomSeed(randomSeed);
          break;
        }
        case "motedelay":
          maxMoteStartupDelay = Integer.parseInt(element.getText()) * MILLISECOND;
          break;
        case "motedelay_us":
          maxMoteStartupDelay = Integer.parseInt(element.getText());
          break;
        case "radiomedium": {
          String radioMediumClassName = element.getText().trim();
          currentRadioMedium = MoteInterfaceHandler.createRadioMedium(this, radioMediumClassName);
          // Show configure simulation dialog
          if (Cooja.isVisualized() && !quick) {
            // FIXME: this should run from the AWT thread.
            if (!setSimConfig(CreateSimDialog.showDialog(getCooja(), getSimConfig()))) {
              return false;
            }
          }
          if (currentRadioMedium == null) {
            throw new MoteType.MoteTypeCreationException("Could not load " + radioMediumClassName);
          }
          // Check if radio medium specific config should be applied
          if (radioMediumClassName.equals(currentRadioMedium.getClass().getName())) {
            currentRadioMedium.setConfigXML(element.getChildren(), Cooja.isVisualized());
          } else {
            logger.info("Radio Medium changed - ignoring radio medium specific config");
          }
          break;
        }
        case "events":
          eventCentral.setConfigXML(this, element.getChildren(), Cooja.isVisualized());
          break;
        case "motetype": {
          String moteTypeClassName = element.getText().trim();
          /* Backwards compatibility: se.sics -> org.contikios */
          if (moteTypeClassName.startsWith("se.sics")) {
            moteTypeClassName = moteTypeClassName.replaceFirst("se\\.sics", "org.contikios");
          }

          var availableMoteTypesObjs = getCooja().getRegisteredMoteTypes();
          String[] availableMoteTypes = new String[availableMoteTypesObjs.size()];
          for (int i = 0; i < availableMoteTypes.length; i++) {
            availableMoteTypes[i] = availableMoteTypesObjs.get(i).getName();
          }

          /* Try to recreate simulation using a different mote type */
          if (Cooja.isVisualized() && !quick) {
            String newClass = (String) JOptionPane.showInputDialog(
                    Cooja.getTopParentContainer(),
                    "The simulation is about to load '" + moteTypeClassName + "'\n" +
                            "You may try to load the simulation using a different mote type.\n",
                    "Loading mote type",
                    JOptionPane.QUESTION_MESSAGE,
                    null,
                    availableMoteTypes,
                    moteTypeClassName
            );
            if (newClass == null) {
              throw new MoteType.MoteTypeCreationException("No mote type class selected");
            }
            if (!newClass.equals(moteTypeClassName)) {
              logger.warn("Changing mote type class: " + moteTypeClassName + " -> " + newClass);
              moteTypeClassName = newClass;
            }
          }

          var moteType = MoteInterfaceHandler.createMoteType(getCooja(), moteTypeClassName);
          if (moteType == null) {
            Class<? extends MoteType> moteTypeClass = null;
            for (int i = 0; i < availableMoteTypes.length; i++) {
              if (moteTypeClassName.equals(availableMoteTypes[i])) {
                moteTypeClass = availableMoteTypesObjs.get(i);
                break;
              }
            }
            assert moteTypeClass != null : "Selected MoteType class is null";
            moteType = moteTypeClass.getConstructor((Class<? extends MoteType>[]) null).newInstance();
          }
          if (!moteType.setConfigXML(this, element.getChildren(), Cooja.isVisualized())) {
            logger.fatal("Mote type was not created: " + element.getText().trim());
            return false;
          }
          addMoteType(moteType);
          break;
        }
        case "mote": {
          MoteType moteType = null;
          for (Element subElement : (Collection<Element>) element.getChildren()) {
            if (subElement.getName().equals("motetype_identifier")) {
              moteType = getMoteType(subElement.getText());
              if (moteType == null) {
                throw new Exception("No mote type '" + subElement.getText() + "' for mote");
              }
              break;
            }
          }
          if (moteType == null) {
            throw new Exception("No mote type specified for mote");
          }
          Mote mote = moteType.generateMote(this);
          if (!mote.setConfigXML(this, element.getChildren(), Cooja.isVisualized())) {
            logger.fatal("Mote was not created: " + element.getText().trim());
            throw new Exception("All motes were not recreated");
          }
          if (getMoteWithID(mote.getID()) != null) {
            logger.warn("Ignoring duplicate mote ID: " + mote.getID());
          } else {
            addMote(mote);
          }
          break;
        }
      }
    }

    if (currentRadioMedium != null) {
      currentRadioMedium.simulationFinishedLoading();
    }

    // Quick load mode only during loading
    this.quick = false;

    setChanged();
    notifyObservers(this);

    return true;
  }

  /**
   * Removes a mote from this simulation
   *
   * @param mote
   *          Mote to remove
   */
  public void removeMote(final Mote mote) {
    invokeSimulationThread(() -> doRemoveMote(mote));
  }

  private void doRemoveMote(Mote mote) {
    motes.remove(mote);
    currentRadioMedium.unregisterMote(mote, Simulation.this);

    /* Dispose mote interface resources */
    mote.removed();
    for (MoteInterface i: mote.getInterfaces().getInterfaces()) {
      i.removed();
    }

    setChanged();
    notifyObservers(mote);

    // Delete all events associated with deleted mote.
    eventQueue.removeIf(ev -> ev instanceof MoteTimeEvent moteTimeEvent && moteTimeEvent.getMote() == mote);

    getCooja().closeMotePlugins(mote);
  }

  /**
   * Called to free resources used by the simulation.
   * This method is called just before the simulation is removed.
   */
  public void removed() {
    if (!isShutdown) {
      commandQueue.add(Command.QUIT);
    }
  }

  /**
   * Adds a mote to this simulation
   *
   * @param mote
   *          Mote to add
   */
  public void addMote(final Mote mote) {
    invokeSimulationThread(new Runnable() {
      @Override
      public void run() {
        if (mote.getInterfaces().getClock() != null) {
          if (maxMoteStartupDelay > 0) {
            mote.getInterfaces().getClock().setDrift(
                - getSimulationTime()
                - randomGenerator.nextInt((int)maxMoteStartupDelay)
            );
          } else {
            mote.getInterfaces().getClock().setDrift(-getSimulationTime());
          }
        }

        motes.add(mote);
        currentRadioMedium.registerMote(mote, Simulation.this);

        /* Notify mote interfaces that node was added */
        for (MoteInterface i: mote.getInterfaces().getInterfaces()) {
          i.added();
        }

        setChanged();
        notifyObservers(mote);
        cooja.updateGUIComponentState();
      }
    });
  }

  /**
   * Returns simulation mote at given list position.
   *
   * @param pos Internal list position of mote
   * @return Mote
   * @see #getMotesCount()
   * @see #getMoteWithID(int)
   */
  public Mote getMote(int pos) {
    return motes.get(pos);
  }

  /**
   * Returns simulation with given ID.
   *
   * @param id ID
   * @return Mote or null
   * @see Mote#getID()
   */
  public Mote getMoteWithID(int id) {
    for (Mote m: motes) {
      if (m.getID() == id) {
        return m;
      }
    }
    return null;
  }

  /**
   * Returns number of motes in this simulation.
   *
   * @return Number of motes
   */
  public int getMotesCount() {
    return motes.size();
  }

  /**
   * Returns all motes in this simulation.
   *
   * @return Motes
   */
  public Mote[] getMotes() {
    Mote[] arr = new Mote[motes.size()];
    motes.toArray(arr);
    return arr;
  }

  /**
   * Returns all mote types in simulation.
   *
   * @return All mote types
   */
  public MoteType[] getMoteTypes() {
    MoteType[] types = new MoteType[moteTypes.size()];
    moteTypes.toArray(types);
    return types;
  }

  /**
   * Returns mote type with given identifier.
   *
   * @param identifier
   *          Mote type identifier
   * @return Mote type or null if not found
   */
  public MoteType getMoteType(String identifier) {
    for (MoteType moteType : getMoteTypes()) {
      if (moteType.getIdentifier().equals(identifier)) {
        return moteType;
      }
    }
    return null;
  }

  /**
   * Adds given mote type to simulation.
   *
   * @param newMoteType Mote type
   */
  public void addMoteType(MoteType newMoteType) {
    moteTypes.add(newMoteType);

    this.setChanged();
    this.notifyObservers(this);
  }

  /**
   * Remove given mote type from simulation.
   *
   * @param type Mote type
   */
  public void removeMoteType(MoteType type) {
    if (!moteTypes.contains(type)) {
      logger.fatal("Mote type is not registered: " + type);
      return;
    }

    /* Remove motes */
    for (Mote m: getMotes()) {
      if (m.getType() == type) {
        removeMote(m);
      }
    }

    moteTypes.remove(type);
    this.setChanged();
    this.notifyObservers(this);
  }

  /**
   * Limit simulation speed to given ratio.
   * This method may be called from outside the simulation thread.
   * @param newSpeedLimit
   */
  public void setSpeedLimit(final Double newSpeedLimit) {
    invokeSimulationThread(new Runnable() {
      @Override
      public void run() {
        if (newSpeedLimit == null) {
          speedLimitNone = true;
          return;
        }

        speedLimitNone = false;
        speedLimitLastRealtime = System.currentTimeMillis();
        speedLimitLastSimtime = getSimulationTimeMillis();
        speedLimit = newSpeedLimit;

        if (delayEvent.isScheduled()) {
          delayEvent.remove();
        }
        scheduleEvent(delayEvent, currentSimulationTime);
        Simulation.this.setChanged();
        Simulation.this.notifyObservers(this);
      }
    });
  }

  /**
   * @return Max simulation speed ratio. Returns null if no limit.
   */
  public Double getSpeedLimit() {
    if (speedLimitNone) {
      return null;
    }
    return speedLimit;
  }

  /**
   * Returns current simulation time.
   *
   * @return Simulation time (microseconds)
   */
  public long getSimulationTime() {
    return currentSimulationTime;
  }

  /**
   * Returns current simulation time rounded to milliseconds.
   *
   * @see #getSimulationTime()
   * @return Time rounded to milliseconds
   */
  public long getSimulationTimeMillis() {
    return currentSimulationTime / MILLISECOND;
  }

  /**
   * Return the actual time value corresponding to an argument which
   * is a simulation time value in microseconds.
   *
   * @return Actual time (microseconds)
   */
  public long convertSimTimeToActualTime(long simTime) {
    return simTime + lastStartRealTime * 1000;
  }

  /**
   * Changes radio medium of this simulation to the given.
   *
   * @param radioMedium
   *          New radio medium
   */
  public void setRadioMedium(RadioMedium radioMedium) {
    // Remove current radio medium from observing motes
    if (currentRadioMedium != null) {
      for (Mote mote : motes) {
        currentRadioMedium.unregisterMote(mote, this);
      }
    }

    // Change current radio medium to new one
    if (radioMedium == null) {
      logger.fatal("Radio medium could not be created.");
      return;
    }
    this.currentRadioMedium = radioMedium;

    // Add all current motes to the new radio medium
    for (Mote mote : motes) {
      currentRadioMedium.registerMote(mote, this);
    }
  }

  /**
   * Get currently used radio medium.
   *
   * @return Currently used radio medium
   */
  public RadioMedium getRadioMedium() {
    return currentRadioMedium;
  }

  /**
   * Return true is simulation is running.
   *
   * @return True if simulation is running
   */
  public boolean isRunning() {
    return isRunning;
  }

  /**
   * Return true is simulation is runnable.
   *
   * @return True if simulation is runnable
   */
  public boolean isRunnable() {
    return isRunning || !eventQueue.isEmpty();
  }

  /**
   * Get current simulation title (short description).
   *
   * @return Title
   */
  public String getTitle() {
    return title;
  }

  /**
   * Set simulation title.
   *
   * @param title
   *          New title
   */
  public void setTitle(String title) {
    this.title = title;
  }
}
