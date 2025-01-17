/*
 * Copyright (c) 2022, Research Institutes of Sweden. All rights
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
 */

package org.contikios.cooja;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dialog;
import java.awt.Dimension;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.InputEvent;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.SynchronousQueue;
import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.ButtonGroup;
import javax.swing.DefaultDesktopManager;
import javax.swing.InputMap;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JComponent;
import javax.swing.JDesktopPane;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JSeparator;
import javax.swing.JTabbedPane;
import javax.swing.JTextPane;
import javax.swing.JToggleButton;
import javax.swing.JToolBar;
import javax.swing.KeyStroke;
import javax.swing.RepaintManager;
import javax.swing.SwingWorker;
import javax.swing.Timer;
import javax.swing.ToolTipManager;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.event.MenuEvent;
import javax.swing.event.MenuListener;
import javax.swing.filechooser.FileFilter;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.contikios.cooja.dialogs.AddMoteDialog;
import org.contikios.cooja.dialogs.BufferSettings;
import org.contikios.cooja.dialogs.CreateSimDialog;
import org.contikios.cooja.dialogs.ExternalToolsDialog;
import org.contikios.cooja.dialogs.MessageList;
import org.contikios.cooja.dialogs.MessageListUI;
import org.contikios.cooja.dialogs.ProjectDirectoriesDialog;
import org.contikios.cooja.plugins.MoteTypeInformation;
import org.contikios.cooja.plugins.SimInformation;
import org.contikios.cooja.util.ScnObservable;
import org.jdom.Element;

/** The graphical user interface for Cooja. */
public class GUI {
  private static final Logger logger = LogManager.getLogger(GUI.class);
  static final String WINDOW_TITLE = "Cooja: The Contiki Network Simulator";

  static JFrame frame;
  final JDesktopPane myDesktopPane;
  private static JProgressBar PROGRESS_BAR = null;
  private static final ArrayList<String> PROGRESS_WARNINGS = new ArrayList<>();

  private final JTextPane quickHelpTextPane;
  private final JMenu menuMoteTypeClasses;
  private final JMenu menuMoteTypes;

  private final ArrayList<GUIAction> guiActions = new ArrayList<>();

  /** Listener for the toolbar. */
  private final ToolbarListener toolbarListener;

  final ScnObservable moteHighlightObservable;

  final ScnObservable moteRelationObservable;
  private final Cooja cooja;
  boolean hasFileHistoryChanged;

  public GUI(Cooja cooja) {
    this.cooja = cooja;
    moteHighlightObservable = new ScnObservable();
    moteRelationObservable = new ScnObservable();
    myDesktopPane = new JDesktopPane() {
      @Override
      public void setBounds(int x, int y, int w, int h) {
        super.setBounds(x, y, w, h);
        updateDesktopSize(this);
      }
      @Override
      public void remove(Component c) {
        super.remove(c);
        updateDesktopSize(this);
      }
      @Override
      public Component add(Component comp) {
        Component c = super.add(comp);
        updateDesktopSize(this);
        return c;
      }
    };
    myDesktopPane.setDesktopManager(new DefaultDesktopManager() {
      @Override
      public void endResizingFrame(JComponent f) {
        super.endResizingFrame(f);
        updateDesktopSize(myDesktopPane);
      }
      @Override
      public void endDraggingFrame(JComponent f) {
        super.endDraggingFrame(f);
        updateDesktopSize(myDesktopPane);
      }
    });
    myDesktopPane.setDragMode(JDesktopPane.OUTLINE_DRAG_MODE);
    frame = new JFrame(WINDOW_TITLE);

    // Help panel.
    quickHelpTextPane = new JTextPane();
    quickHelpTextPane.setContentType("text/html");
    quickHelpTextPane.setEditable(false);
    quickHelpTextPane.setVisible(false);

    // Print a warning when repainting outside EDT with: gradlew run -Dcooja.debug.repaint=true
    if (System.getProperty("debug.repaint") != null) {
      RepaintManager.setCurrentManager(new RepaintManager() {
        public void addDirtyRegion(JComponent comp, int a, int b, int c, int d) {
          if (!java.awt.EventQueue.isDispatchThread()) {
            // Log to console so the thread name is printed along with the complete backtrace.
            logger.warn("Repainting outside EDT", new IllegalStateException("Repainting outside EDT"));
          }
          super.addDirtyRegion(comp, a, b, c, d);
        }
      });
    }

    // Parse current extension configuration.
    try {
      cooja.parseProjectConfig();
    } catch (Cooja.ParseProjectsException e) {
      logger.fatal("Error when loading extensions: " + e.getMessage(), e);
      JOptionPane.showMessageDialog(frame,
              "All Cooja extensions could not load.\n\n" +
                      "To manage Cooja extensions:\n" +
                      "Menu->Settings->Cooja extensions",
              "Reconfigure Cooja extensions", JOptionPane.INFORMATION_MESSAGE);
      showErrorDialog("Cooja extensions load error", e, false);
    }

    // Start all standard GUI plugins
    for (var pluginClass : cooja.getRegisteredPlugins()) {
      int pluginType = pluginClass.getAnnotation(PluginType.class).value();
      if (pluginType == PluginType.COOJA_STANDARD_PLUGIN) {
        cooja.tryStartPlugin(pluginClass, null, null);
      }
    }

    frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);

    // Menu bar.
    menuMoteTypeClasses = new JMenu("Create new mote type");
    menuMoteTypeClasses.setMnemonic(KeyEvent.VK_C);
    menuMoteTypes = new JMenu("Add motes");
    menuMoteTypes.setMnemonic(KeyEvent.VK_A);
    // Create simulation control toolbar.
    var toolBar = new JToolBar("Simulation Control");
    var startButton = new JToggleButton("Start/Pause");
    startButton.setText("Start/Pause");
    startButton.setToolTipText("Start");
    toolBar.add(startButton);
    var stepButton = new JButton("Step");
    stepButton.setToolTipText("Step");
    toolBar.add(stepButton);
    var reloadButton = new JButton("Reload");
    reloadButton.setToolTipText("Reload");
    toolBar.add(reloadButton);
    toolBar.addSeparator();
    toolBar.add(new JLabel("Speed limit:"));
    var pane = new JPanel(new GridBagLayout());
    var group = new ButtonGroup();
    var radioConstraints = new GridBagConstraints();
    radioConstraints.fill = GridBagConstraints.HORIZONTAL;
    var slowCrawlSpeedButton = new JRadioButton("0.01X");
    slowCrawlSpeedButton.setToolTipText("1%");
    pane.add(slowCrawlSpeedButton, radioConstraints);
    group.add(slowCrawlSpeedButton);
    var crawlSpeedButton = new JRadioButton("0.1X");
    crawlSpeedButton.setToolTipText("10%");
    pane.add(crawlSpeedButton, radioConstraints);
    group.add(crawlSpeedButton);
    var normalSpeedButton = new JRadioButton("1X");
    normalSpeedButton.setToolTipText("100%");
    pane.add(normalSpeedButton, radioConstraints);
    group.add(normalSpeedButton);
    var doubleSpeedButton = new JRadioButton("2X");
    doubleSpeedButton.setToolTipText("200%");
    pane.add(doubleSpeedButton, radioConstraints);
    group.add(doubleSpeedButton);
    var superSpeedButton = new JRadioButton("20X");
    superSpeedButton.setToolTipText("2000%");
    pane.add(superSpeedButton, radioConstraints);
    group.add(superSpeedButton);
    var unlimitedSpeedButton = new JRadioButton("Unlimited");
    superSpeedButton.setToolTipText("Unlimited");
    pane.add(unlimitedSpeedButton, radioConstraints);
    group.add(unlimitedSpeedButton);
    toolBar.add(pane);
    toolBar.addSeparator();
    final var simulationTime = new JLabel("Time:");
    toolBar.add(simulationTime);
    toolBar.setMinimumSize(toolBar.getSize());
    var container = new JPanel(new BorderLayout());
    container.add(toolBar, BorderLayout.PAGE_START);

    toolbarListener = new ToolbarListener() {
      private final Timer updateTimer = new Timer(500, e -> {
        final var sim = cooja.getSimulation();
        simulationTime.setText(getTimeString(sim));
      });

      @Override
      public void itemStateChanged(ItemEvent e) {
        final var sim = cooja.getSimulation();
        // Simulation is null when resetting the state of startButton after closing simulation.
        if (sim == null) {
          return;
        }
        switch (e.getStateChange()) {
          case ItemEvent.SELECTED -> {
            sim.startSimulation();
            stepButton.setEnabled(false);
          }
          case ItemEvent.DESELECTED -> {
            sim.stopSimulation();
            stepButton.setEnabled(true);
          }
        }
        updateToolbar(false);
      }

      @Override
      public void updateToolbar(boolean stoppedSimulation) {
        // Ensure the start button can be pressed if this update was from stopping the simulation.
        if (stoppedSimulation) {
          startButton.setSelected(false);
          updateTimer.stop();
        }
        final var sim = cooja.getSimulation();
        simulationTime.setText(getTimeString(sim));
        var hasSim = sim != null;
        var state = hasSim && !sim.isRunning() && sim.isRunnable();
        startButton.setEnabled(hasSim && sim.isRunnable());
        startButton.setSelected(hasSim && sim.isRunning());
        stepButton.setEnabled(state);
        reloadButton.setEnabled(hasSim);
        slowCrawlSpeedButton.setEnabled(hasSim);
        crawlSpeedButton.setEnabled(hasSim);
        normalSpeedButton.setEnabled(hasSim);
        doubleSpeedButton.setEnabled(hasSim);
        superSpeedButton.setEnabled(hasSim);
        unlimitedSpeedButton.setEnabled(hasSim);
        if (hasSim) {
          Double speed = sim.getSpeedLimit();
          if (speed == null) {
            unlimitedSpeedButton.setSelected(true);
          } else if (speed == 0.01) {
            slowCrawlSpeedButton.setSelected(true);
          } else if (speed == 0.1) {
            crawlSpeedButton.setSelected(true);
          } else if (speed == 1.0) {
            normalSpeedButton.setSelected(true);
          } else if (speed == 2.0) {
            doubleSpeedButton.setSelected(true);
          } else if (speed == 20.0) {
            superSpeedButton.setSelected(true);
          }
        } else {
          startButton.setSelected(false);
        }
        // Start timer after updating the UI states.
        if (hasSim && sim.isRunning() && !stoppedSimulation && !updateTimer.isRunning()) {
          updateTimer.start();
        }
      }
      private static final long TIME_SECOND = 1000 * Simulation.MILLISECOND;
      private static final long TIME_MINUTE = 60 * TIME_SECOND;
      private static final long TIME_HOUR = 60 * TIME_MINUTE;

      public static String getTimeString(Simulation sim) {
        if (sim == null) {
          return "Time:";
        }
        long t = sim.getSimulationTime();
        long h = t / TIME_HOUR;
        t -= (t / TIME_HOUR) * TIME_HOUR;
        long m = t / TIME_MINUTE;
        t -= (t / TIME_MINUTE) * TIME_MINUTE;
        long s = t / TIME_SECOND;
        t -= (t / TIME_SECOND) * TIME_SECOND;
        long ms = t / Simulation.MILLISECOND;
        if (h > 0) {
          return String.format("Time: %d:%02d:%02d.%03d", h, m, s, ms);
        }
        return String.format("Time: %02d:%02d.%03d", m, s, ms);
      }
    };

    startButton.addItemListener(toolbarListener);
    final var buttonAction = new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        var source = e.getSource();
        if (source == stepButton) {
          cooja.getSimulation().stepMillisecondSimulation();
        } else if (source == reloadButton) {
          reloadCurrentSimulation(cooja.getSimulation().getRandomSeed());
        }
        java.awt.EventQueue.invokeLater(() -> toolbarListener.updateToolbar(source == reloadButton));
      }
    };
    stepButton.addActionListener(buttonAction);
    reloadButton.addActionListener(buttonAction);
    final var speedListener = new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        switch (e.getActionCommand()) {
          case "0.01X" -> cooja.getSimulation().setSpeedLimit(0.01);
          case "0.1X" -> cooja.getSimulation().setSpeedLimit(0.1);
          case "1X" -> cooja.getSimulation().setSpeedLimit(1.0);
          case "2X" -> cooja.getSimulation().setSpeedLimit(2.0);
          case "20X" -> cooja.getSimulation().setSpeedLimit(20.0);
          case "Unlimited" -> cooja.getSimulation().setSpeedLimit(null);
        }
      }
    };
    slowCrawlSpeedButton.addActionListener(speedListener);
    crawlSpeedButton.addActionListener(speedListener);
    normalSpeedButton.addActionListener(speedListener);
    doubleSpeedButton.addActionListener(speedListener);
    superSpeedButton.addActionListener(speedListener);
    unlimitedSpeedButton.addActionListener(speedListener);
    final var newSimulationAction = new GUIAction("New simulation...", KeyEvent.VK_N, KeyStroke.getKeyStroke(KeyEvent.VK_N, InputEvent.CTRL_DOWN_MASK)) {
      @Override
      public void actionPerformed(ActionEvent e) {
        if (!cooja.doRemoveSimulation(true)) {
          return;
        }

        var cfg = CreateSimDialog.showDialog(cooja, new Simulation.SimConfig(null, null,
                false, 123456, 1000));
        if (cfg == null) return;
        var sim = new Simulation(cooja, cooja.configuration.logDir(), cfg.randomSeed());
        boolean ok;
        try {
          ok = sim.setSimConfig(cfg);
        } catch (MoteType.MoteTypeCreationException ex) {
          ok = false;
        }
        if (ok) {
          // Start GUI plugins.
          for (var pluginClass : cooja.getRegisteredPlugins()) {
            int type = pluginClass.getAnnotation(PluginType.class).value();
            if (type == PluginType.SIM_STANDARD_PLUGIN) {
              cooja.tryStartPlugin(pluginClass, sim, null);
            }
          }
          cooja.setSimulation(sim);
        }
      }
      @Override
      public boolean shouldBeEnabled() {
        return true;
      }
    };
    final var closeSimulationAction = new GUIAction("Close simulation", KeyEvent.VK_C) {
      @Override
      public void actionPerformed(ActionEvent e) {
        cooja.doRemoveSimulation(true);
      }
      @Override
      public boolean shouldBeEnabled() {
        return cooja.getSimulation() != null;
      }
    };
    final var reloadSimulationAction = new GUIAction("Reload with same random seed", KeyEvent.VK_K, KeyStroke.getKeyStroke(KeyEvent.VK_R, InputEvent.CTRL_DOWN_MASK)) {
      @Override
      public void actionPerformed(ActionEvent e) {
        if (cooja.getSimulation() == null) {
          // Reload last opened simulation.
          final File file = getLastOpenedFile();
          doLoadConfigAsync(true, file);
          return;
        }
        reloadCurrentSimulation(cooja.getSimulation().getRandomSeed());
      }
      @Override
      public boolean shouldBeEnabled() {
        return true;
      }
    };
    final var reloadRandomSimulationAction = new GUIAction("Reload with new random seed", KeyEvent.VK_N, KeyStroke.getKeyStroke(KeyEvent.VK_R, InputEvent.SHIFT_DOWN_MASK | InputEvent.CTRL_DOWN_MASK)) {
      @Override
      public void actionPerformed(ActionEvent e) {
        if (cooja.getSimulation() != null) {
          reloadCurrentSimulation(cooja.getSimulation().getRandomSeed() + 1);
        }
      }
      @Override
      public boolean shouldBeEnabled() {
        return cooja.getSimulation() != null;
      }
    };
    final var saveSimulationAction = new GUIAction("Save simulation as...", KeyEvent.VK_S) {
      @Override
      public void actionPerformed(ActionEvent e) {
        doSaveConfig();
      }
      @Override
      public boolean shouldBeEnabled() {
        return cooja.getSimulation() != null;
      }
    };
    final var exitCoojaAction = new GUIAction("Exit", 'x') {
      @Override
      public void actionPerformed(ActionEvent e) {
        cooja.doQuit(true);
      }
      @Override
      public boolean shouldBeEnabled() {
        return true;
      }
    };
    final var startStopSimulationAction = new GUIAction("Start simulation", KeyStroke.getKeyStroke(KeyEvent.VK_S, InputEvent.CTRL_DOWN_MASK)) {
      @Override
      public void actionPerformed(ActionEvent e) {
        // Start/Stop current simulation.
        Simulation s = cooja.getSimulation();
        if (s == null) {
          return;
        }
        if (s.isRunning()) {
          s.stopSimulation();
        } else {
          s.startSimulation();
        }
      }
      @Override
      public void setEnabled(boolean newValue) {
        if (cooja.getSimulation() == null) {
          putValue(NAME, "Start simulation");
        } else if (cooja.getSimulation().isRunning()) {
          putValue(NAME, "Pause simulation");
        } else {
          putValue(NAME, "Start simulation");
        }
        super.setEnabled(newValue);
      }
      @Override
      public boolean shouldBeEnabled() {
        return cooja.getSimulation() != null && cooja.getSimulation().isRunnable();
      }
    };
    final var removeAllMotesAction = new GUIAction("Remove all motes") {
      @Override
      public void actionPerformed(ActionEvent e) {
        Simulation s = cooja.getSimulation();
        s.stopSimulation();

        while (s.getMotesCount() > 0) {
          s.removeMote(cooja.getSimulation().getMote(0));
        }
      }
      @Override
      public boolean shouldBeEnabled() {
        Simulation s = cooja.getSimulation();
        return s != null && s.getMotesCount() > 0;
      }
    };
    final var showBufferSettingsAction = new GUIAction("Buffer sizes...") {
      @Override
      public void actionPerformed(ActionEvent e) {
        if (cooja.getSimulation() == null) {
          return;
        }
        BufferSettings.showDialog(cooja.getSimulation());
      }
      @Override
      public boolean shouldBeEnabled() {
        return cooja.getSimulation() != null;
      }
    };

    // Prepare GUI actions.
    guiActions.add(newSimulationAction);
    guiActions.add(closeSimulationAction);
    guiActions.add(reloadSimulationAction);
    guiActions.add(reloadRandomSimulationAction);
    guiActions.add(saveSimulationAction);
    guiActions.add(exitCoojaAction);
    guiActions.add(startStopSimulationAction);
    guiActions.add(removeAllMotesAction);
    guiActions.add(showBufferSettingsAction);

    // Menus.
    JMenuBar menuBar = new JMenuBar();
    JMenu fileMenu = new JMenu("File");
    JMenu simulationMenu = new JMenu("Simulation");
    JMenu motesMenu = new JMenu("Motes");
    final JMenu toolsMenu = new JMenu("Tools");
    JMenu settingsMenu = new JMenu("Settings");
    JMenu helpMenu = new JMenu("Help");
    var menuOpenSimulation = new JMenu("Open simulation");
    menuOpenSimulation.setMnemonic(KeyEvent.VK_O);

    menuBar.add(fileMenu);
    menuBar.add(simulationMenu);
    menuBar.add(motesMenu);
    menuBar.add(toolsMenu);
    menuBar.add(settingsMenu);
    menuBar.add(helpMenu);

    fileMenu.setMnemonic(KeyEvent.VK_F);
    simulationMenu.setMnemonic(KeyEvent.VK_S);
    motesMenu.setMnemonic(KeyEvent.VK_M);
    toolsMenu.setMnemonic(KeyEvent.VK_T);
    helpMenu.setMnemonic(KeyEvent.VK_H);

    // File menu.
    fileMenu.addMenuListener(new MenuListener() {
      @Override
      public void menuSelected(MenuEvent e) {
        updateGUIComponentState();
        if (!hasFileHistoryChanged) {
          return;
        }
        hasFileHistoryChanged = false;

        // Reconfigure submenu.
        menuOpenSimulation.removeAll();
        JMenu reconfigureMenu = new JMenu("Open and Reconfigure");
        JMenuItem browseItem2 = new JMenuItem("Browse...");
        browseItem2.addActionListener(e1 -> doLoadConfigAsync(false, null));
        reconfigureMenu.add(browseItem2);
        reconfigureMenu.add(new JSeparator());
        File[] openFilesHistory = getFileHistory();
        populateMenuWithHistory(reconfigureMenu, false, openFilesHistory);

        // Open menu.
        JMenuItem browseItem = new JMenuItem("Browse...");
        browseItem.addActionListener(e1 -> doLoadConfigAsync(true, null));
        menuOpenSimulation.add(browseItem);
        menuOpenSimulation.add(new JSeparator());
        menuOpenSimulation.add(reconfigureMenu);
        menuOpenSimulation.add(new JSeparator());
        populateMenuWithHistory(menuOpenSimulation, true, openFilesHistory);
      }
      @Override
      public void menuDeselected(MenuEvent e) {}

      @Override
      public void menuCanceled(MenuEvent e) {}
    });

    fileMenu.add(new JMenuItem(newSimulationAction));
    fileMenu.add(menuOpenSimulation);
    fileMenu.add(new JMenuItem(closeSimulationAction));

    hasFileHistoryChanged = true;

    fileMenu.add(new JMenuItem(saveSimulationAction));
    fileMenu.addSeparator();
    fileMenu.add(new JMenuItem(exitCoojaAction));

    // Simulation menu.
    simulationMenu.addMenuListener(new MenuListener() {
      @Override
      public void menuSelected(MenuEvent e) {
        updateGUIComponentState();
      }
      @Override
      public void menuDeselected(MenuEvent e) {
      }
      @Override
      public void menuCanceled(MenuEvent e) {
      }
    });
    simulationMenu.add(new JMenuItem(startStopSimulationAction));

    JMenuItem reloadSimulationMenuItem = new JMenu("Reload simulation");
    reloadSimulationMenuItem.add(new JMenuItem(reloadSimulationAction));
    reloadSimulationMenuItem.add(new JMenuItem(reloadRandomSimulationAction));
    simulationMenu.add(reloadSimulationMenuItem);

    var guiAction = new StartPluginGUIAction("Information...");
    var menuItem = new JMenuItem(guiAction);
    guiActions.add(guiAction);
    menuItem.setMnemonic(KeyEvent.VK_I);
    menuItem.putClientProperty("class", SimInformation.class);
    simulationMenu.add(menuItem);

    // Mote type menu
    motesMenu.addMenuListener(new MenuListener() {
      @Override
      public void menuSelected(MenuEvent e) {
        updateGUIComponentState();
      }
      @Override
      public void menuDeselected(MenuEvent e) {}
      @Override
      public void menuCanceled(MenuEvent e) {}
    });
    final var guiEventHandler = new GUIEventHandler();
    // Mote type classes sub menu
    menuMoteTypeClasses.addMenuListener(new MenuListener() {
      @Override
      public void menuSelected(MenuEvent e) {
        // Clear menu and recreate items.
        menuMoteTypeClasses.removeAll();
        for (var moteTypeClass : cooja.getRegisteredMoteTypes()) {
          // Sort mote types according to abstraction level.
          String abstractionLevelDescription = Cooja.getAbstractionLevelDescriptionOf(moteTypeClass);
          if(abstractionLevelDescription == null) {
            abstractionLevelDescription = "[unknown cross-level]";
          }

          // Check if abstraction description already exists.
          JSeparator abstractionLevelSeparator = null;
          for (Component component: menuMoteTypeClasses.getMenuComponents()) {
            if (!(component instanceof JSeparator)) {
              continue;
            }
            JSeparator existing = (JSeparator) component;
            if (abstractionLevelDescription.equals(existing.getToolTipText())) {
              abstractionLevelSeparator = existing;
              break;
            }
          }
          if (abstractionLevelSeparator == null) {
            abstractionLevelSeparator = new JSeparator();
            abstractionLevelSeparator.setToolTipText(abstractionLevelDescription);
            menuMoteTypeClasses.add(abstractionLevelSeparator);
          }

          String description = Cooja.getDescriptionOf(moteTypeClass);
          var menuItem = new JMenuItem(description + "...");
          menuItem.setActionCommand("create mote type");
          menuItem.putClientProperty("class", moteTypeClass);
          menuItem.addActionListener(guiEventHandler);

          // Add new item directly after cross level separator.
          for (int i=0; i < menuMoteTypeClasses.getMenuComponentCount(); i++) {
            if (menuMoteTypeClasses.getMenuComponent(i) == abstractionLevelSeparator) {
              menuMoteTypeClasses.add(menuItem, i+1);
              break;
            }
          }
        }
      }

      @Override
      public void menuDeselected(MenuEvent e) {}

      @Override
      public void menuCanceled(MenuEvent e) {}
    });

    // Mote menu
    motesMenu.addMenuListener(new MenuListener() {
      @Override
      public void menuSelected(MenuEvent e) {
        updateGUIComponentState();
      }
      @Override
      public void menuDeselected(MenuEvent e) {}
      @Override
      public void menuCanceled(MenuEvent e) {}
    });

    // Mote types sub menu
    menuMoteTypes.addMenuListener(new MenuListener() {
      @Override
      public void menuSelected(MenuEvent e) {
        // Clear menu
        menuMoteTypes.removeAll();
        if (cooja.getSimulation() != null) {
          // Recreate menu items
          for (MoteType moteType : cooja.getSimulation().getMoteTypes()) {
            var menuItem = new JMenuItem(moteType.getDescription());
            menuItem.setActionCommand("add motes");
            menuItem.setToolTipText(Cooja.getDescriptionOf(moteType.getClass()));
            menuItem.putClientProperty("motetype", moteType);
            menuItem.addActionListener(guiEventHandler);
            menuMoteTypes.add(menuItem);
          }
          if(cooja.getSimulation().getMoteTypes().length > 0) {
            menuMoteTypes.add(new JSeparator());
          }
        }
        menuMoteTypes.add(menuMoteTypeClasses);
      }

      @Override
      public void menuDeselected(MenuEvent e) {}

      @Override
      public void menuCanceled(MenuEvent e) {}
    });
    motesMenu.add(menuMoteTypes);

    guiAction = new StartPluginGUIAction("Mote types...");
    menuItem = new JMenuItem(guiAction);
    guiActions.add(guiAction);
    menuItem.putClientProperty("class", MoteTypeInformation.class);

    motesMenu.add(menuItem);

    motesMenu.add(new JMenuItem(removeAllMotesAction));

    // Tools menu.
    toolsMenu.addMenuListener(new MenuListener() {
      private final ActionListener menuItemListener = e -> {
        Object pluginClass = ((JMenuItem)e.getSource()).getClientProperty("class");
        Object mote = ((JMenuItem)e.getSource()).getClientProperty("mote");
        cooja.tryStartPlugin((Class<? extends Plugin>) pluginClass, cooja.getSimulation(), (Mote)mote);
      };
      private JMenuItem createMenuItem(Class<? extends Plugin> newPluginClass) {
        String description = Cooja.getDescriptionOf(newPluginClass);
        JMenuItem menuItem = new JMenuItem(description + "...");
        menuItem.putClientProperty("class", newPluginClass);
        menuItem.addActionListener(menuItemListener);
        // Only enable items when there is a simulation, otherwise the user gets a dialog with a backtrace.
        menuItem.setEnabled(cooja.getSimulation() != null);
        return menuItem;
      }

      @Override
      public void menuSelected(MenuEvent e) {
        // Populate tools menu.
        toolsMenu.removeAll();

        // Cooja plugins.
        boolean hasCoojaPlugins = false;
        for (Class<? extends Plugin> pluginClass : cooja.getRegisteredPlugins()) {
          int pluginType = pluginClass.getAnnotation(PluginType.class).value();
          if (pluginType != PluginType.COOJA_PLUGIN && pluginType != PluginType.COOJA_STANDARD_PLUGIN) {
            continue;
          }
          toolsMenu.add(createMenuItem(pluginClass));
          hasCoojaPlugins = true;
        }

        // Simulation plugins.
        boolean hasSimPlugins = false;
        for (Class<? extends Plugin> pluginClass : cooja.getRegisteredPlugins()) {
          if (pluginClass.equals(SimInformation.class) || pluginClass.equals(MoteTypeInformation.class)) {
            continue; // Ignore.
          }

          int pluginType = pluginClass.getAnnotation(PluginType.class).value();
          if (pluginType != PluginType.SIM_PLUGIN && pluginType != PluginType.SIM_STANDARD_PLUGIN
                  && pluginType != PluginType.SIM_CONTROL_PLUGIN) {
            continue;
          }

          if (hasCoojaPlugins) {
            hasCoojaPlugins = false;
            toolsMenu.addSeparator();
          }

          toolsMenu.add(createMenuItem(pluginClass));
          hasSimPlugins = true;
        }

        for (Class<? extends Plugin> pluginClass : cooja.getRegisteredPlugins()) {
          int pluginType = pluginClass.getAnnotation(PluginType.class).value();
          if (pluginType != PluginType.MOTE_PLUGIN) {
            continue;
          }

          if (hasSimPlugins) {
            hasSimPlugins = false;
            toolsMenu.addSeparator();
          }

          toolsMenu.add(createMotePluginsSubmenu(pluginClass));
        }
      }
      @Override
      public void menuDeselected(MenuEvent e) {}
      @Override
      public void menuCanceled(MenuEvent e) {}
    });

    // Settings menu
    settingsMenu.addMenuListener(new MenuListener() {
      @Override
      public void menuSelected(MenuEvent e) {
        updateGUIComponentState();
      }
      @Override
      public void menuDeselected(MenuEvent e) {}
      @Override
      public void menuCanceled(MenuEvent e) {}
    });

    menuItem = new JMenuItem("External tools paths...");
    menuItem.setActionCommand("edit paths");
    menuItem.addActionListener(guiEventHandler);
    settingsMenu.add(menuItem);

    menuItem = new JMenuItem("Cooja extensions...");
    menuItem.setActionCommand("manage extensions");
    menuItem.addActionListener(guiEventHandler);
    settingsMenu.add(menuItem);

    settingsMenu.add(new JMenuItem(showBufferSettingsAction));

    // Help.
    var quickHelpScroll = new JScrollPane(quickHelpTextPane, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
    quickHelpScroll.setPreferredSize(new Dimension(200, 0));
    quickHelpScroll.setBorder(BorderFactory.createCompoundBorder(
            BorderFactory.createLineBorder(Color.GRAY),
            BorderFactory.createEmptyBorder(0, 3, 0, 0)));
    quickHelpScroll.setVisible(false);
    var scroll = new JScrollPane(myDesktopPane);
    scroll.setBorder(null);
    container.add(BorderLayout.CENTER, scroll);
    container.add(BorderLayout.EAST, quickHelpScroll);
    loadQuickHelp("GETTING_STARTED");
    final var checkBox = new JCheckBoxMenuItem(new GUIAction("Quick help", KeyStroke.getKeyStroke(KeyEvent.VK_F1, 0)) {
      @Override
      public void actionPerformed(ActionEvent e) {
        if (!(e.getSource() instanceof JCheckBoxMenuItem)) {
          return;
        }
        boolean show = ((JCheckBoxMenuItem) e.getSource()).isSelected();
        quickHelpTextPane.setVisible(show);
        quickHelpScroll.setVisible(show);
        Cooja.setExternalToolsSetting("SHOW_QUICKHELP", Boolean.toString(show));
        frame.getContentPane().revalidate();
        updateDesktopSize(myDesktopPane);
      }

      @Override
      public boolean shouldBeEnabled() {
        return true;
      }
    });
    final var showGettingStartedAction = new GUIAction("Getting started") {
      @Override
      public void actionPerformed(ActionEvent e) {
        loadQuickHelp("GETTING_STARTED");
        if (!checkBox.isSelected()) {
          checkBox.doClick();
        }
      }

      @Override
      public boolean shouldBeEnabled() {
        return true;
      }
    };
    helpMenu.add(new JMenuItem(showGettingStartedAction));
    if (Cooja.getExternalToolsSetting("SHOW_QUICKHELP", "true").equalsIgnoreCase("true")) {
      showGettingStartedAction.actionPerformed(null);
    }
    helpMenu.add(new JMenuItem(new GUIAction("Keyboard shortcuts") {
      @Override
      public void actionPerformed(ActionEvent e) {
        loadQuickHelp("KEYBOARD_SHORTCUTS");
        if (!checkBox.isSelected()) {
          checkBox.doClick();
        }
      }

      @Override
      public boolean shouldBeEnabled() {
        return true;
      }
    }));
    helpMenu.add(checkBox);

    helpMenu.addSeparator();

    menuItem = new JMenuItem("Java version: "
            + System.getProperty("java.version") + " (" + System.getProperty("java.vendor") + ")");
    menuItem.setEnabled(false);
    helpMenu.add(menuItem);
    menuItem = new JMenuItem("System \"os.arch\": " + System.getProperty("os.arch"));
    menuItem.setEnabled(false);
    helpMenu.add(menuItem);
    menuItem = new JMenuItem("System \"sun.arch.data.model\": " + System.getProperty("sun.arch.data.model"));
    menuItem.setEnabled(false);
    helpMenu.add(menuItem);

    frame.setJMenuBar(menuBar);

    // Scrollable desktop.
    myDesktopPane.setOpaque(true);
    frame.setContentPane(container);

    frame.setSize(700, 700);
    frame.setLocationRelativeTo(null);
    frame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosing(WindowEvent e) {
        cooja.doQuit(true);
      }
    });
    frame.addComponentListener(new ComponentAdapter() {
      @Override
      public void componentResized(ComponentEvent e) {
        updateDesktopSize(myDesktopPane);
      }
    });

    int framePosX = Integer.parseInt(Cooja.getExternalToolsSetting("FRAME_POS_X", "0"));
    int framePosY = Integer.parseInt(Cooja.getExternalToolsSetting("FRAME_POS_Y", "0"));
    int frameWidth = Integer.parseInt(Cooja.getExternalToolsSetting("FRAME_WIDTH", "0"));
    int frameHeight = Integer.parseInt(Cooja.getExternalToolsSetting("FRAME_HEIGHT", "0"));
    String frameScreen = Cooja.getExternalToolsSetting("FRAME_SCREEN", "");

    // Restore position to the same graphics device.
    GraphicsDevice device = null;
    for (var gd : GraphicsEnvironment.getLocalGraphicsEnvironment().getScreenDevices()) {
      if (gd.getIDstring().equals(frameScreen)) {
        device = gd;
        break;
      }
    }

    // Restore frame size and position.
    if (device != null) {
      if (frameWidth == Integer.MAX_VALUE && frameHeight == Integer.MAX_VALUE) {
        frame.setLocation(device.getDefaultConfiguration().getBounds().getLocation());
        frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
      } else if (frameWidth > 0 && frameHeight > 0) {
        // Ensure Cooja is visible on screen.
        boolean intersects =
                device.getDefaultConfiguration().getBounds().intersects(
                        new Rectangle(framePosX, framePosY, frameWidth, frameHeight));
        if (intersects) {
          frame.setLocation(framePosX, framePosY);
          frame.setSize(frameWidth, frameHeight);
        }
      }
    }
    updateProgress(false);
    frame.setVisible(true);
  }

  public void loadQuickHelp(final Object obj) {
    String help;
    if (obj instanceof HasQuickHelp) {
      help = ((HasQuickHelp) obj).getQuickHelp();
    } else {
      String key;
      if (obj instanceof String) {
        key = (String) obj;
      } else {
        key = obj.getClass().getName();
      }
      help = switch (key) {
        case "KEYBOARD_SHORTCUTS" -> "<b>Keyboard shortcuts</b><br>" +
                "<br><i>Ctrl+N:</i> New simulation" +
                "<br><i>Ctrl+S:</i> Start/pause simulation" +
                "<br><i>Ctrl+R:</i> Reload current simulation. If no simulation exists, the last used simulation config is loaded" +
                "<br><i>Ctrl+Shift+R:</i> Reload current simulation with another random seed" +
                "<br>" +
                "<br><i>F1:</i> Toggle quick help";
        case "GETTING_STARTED" -> "<b>Getting started</b><br>" +
                "<br>" +
                "<br><i>F1:</i> Toggle quick help</i>";
        default -> null;
      };
    }

    if (help != null) {
      quickHelpTextPane.setText("<html>" + help + "</html>");
    } else {
      quickHelpTextPane.setText("<html><b>" + Cooja.getDescriptionOf(obj) +"</b>" +
              "<p>No help available</html>");
    }
    quickHelpTextPane.setCaretPosition(0);
  }

  public JMenu createMotePluginsSubmenu(Class<? extends Plugin> pluginClass) {
    JMenu menu = new JMenu(Cooja.getDescriptionOf(pluginClass));
    if (cooja.getSimulation() == null || cooja.getSimulation().getMotesCount() == 0) {
      menu.setEnabled(false);
      return menu;
    }

    ActionListener menuItemListener = e -> {
      Object pluginClass1 = ((JMenuItem)e.getSource()).getClientProperty("class");
      Object mote = ((JMenuItem)e.getSource()).getClientProperty("mote");
      cooja.tryStartPlugin((Class<? extends Plugin>) pluginClass1, cooja.getSimulation(), (Mote)mote);
    };

    final int MAX_PER_ROW = 30;
    final int MAX_COLUMNS = 5;

    int added = 0;
    for (var mote : cooja.getSimulation().getMotes()) {
      if (!Cooja.isMotePluginCompatible(pluginClass, mote)) {
        continue;
      }
      JMenuItem menuItem = new JMenuItem(mote.toString() + "...");
      menuItem.putClientProperty("class", pluginClass);
      menuItem.putClientProperty("mote", mote);
      menuItem.addActionListener(menuItemListener);
      menu.add(menuItem);
      added++;
      if (added == MAX_PER_ROW) {
        menu.getPopupMenu().setLayout(new GridLayout(MAX_PER_ROW, MAX_COLUMNS));
      }
      if (added >= MAX_PER_ROW*MAX_COLUMNS) {
        break;
      }
    }
    if (added == 0) {
      menu.setEnabled(false);
    }
    return menu;
  }

  public JMenu createMotePluginsSubmenu(Mote mote) {
    JMenu menuMotePlugins = new JMenu("Mote tools for " + mote);
    for (var motePluginClass: cooja.getMenuMotePluginClasses()) {
      if (!Cooja.isMotePluginCompatible(motePluginClass, mote)) {
        continue;
      }
      var menuItem = new JMenuItem(new StartPluginGUIAction(Cooja.getDescriptionOf(motePluginClass) + "..."));
      menuItem.putClientProperty("class", motePluginClass);
      menuItem.putClientProperty("mote", mote);
      menuMotePlugins.add(menuItem);
    }
    return menuMotePlugins;
  }

  private void populateMenuWithHistory(JMenu menu, final boolean quick, File[] openFilesHistory) {
    JMenuItem lastItem;
    int index = 0;
    for (File file: openFilesHistory) {
      if (index < 10) {
        char mnemonic = (char) ('0' + (++index % 10));
        lastItem = new JMenuItem(mnemonic + " " + file.getName());
        lastItem.setMnemonic(mnemonic);
      } else {
        lastItem = new JMenuItem(file.getName());
      }
      final File f = file;
      lastItem.addActionListener(e -> doLoadConfigAsync(quick, f));
      lastItem.putClientProperty("file", file);
      lastItem.setToolTipText(file.getAbsolutePath());
      menu.add(lastItem);
    }
  }

  /**
   * Reload currently configured simulation, which may include recompiling Contiki-NG.
   * @param seed Seed to use for reloaded simulation.
   */
  public void reloadCurrentSimulation(long seed) {
    if (warnMemory()) {
      return;
    }
    var worker = createLoadSimWorker(null, true, false, seed);
    if (worker == null) return;
    worker.execute();
  }

  /**
   * Load a simulation configuration file from disk asynchronously.
   *
   * @param quick Quick-load simulation
   * @param file Configuration file to load, if null a dialog will appear
   */
  private void doLoadConfigAsync(final boolean quick, File file) {
    // Warn about memory usage.
    if (warnMemory()) {
      return;
    }

    final var cfgFile = validateFileOrSelectNew(file);
    if (cfgFile == null) return;

    var worker = createLoadSimWorker(cfgFile, quick, false, null);
    if (worker == null) return;
    worker.execute();
  }

  /**
   * Load a simulation configuration file from disk and return the simulation.
   *
   * @param configFile Configuration file to load, reloads current sim if null
   * @param rewriteCsc Rewrite simulation config
   * @param manualRandomSeed The random seed to use for the simulation
   * @return The simulation
   */
  Simulation doLoadConfig(File configFile, boolean rewriteCsc, Long manualRandomSeed) {
    final var worker = new Cooja.RunnableInEDT<SwingWorker<Simulation, Cooja.SimulationCreationException>>() {
      @Override
      public SwingWorker<Simulation, Cooja.SimulationCreationException> work() {
        return createLoadSimWorker(configFile, true, rewriteCsc, manualRandomSeed);
      }
    }.invokeAndWait();
    worker.execute();
    try {
      return worker.get();
    } catch (CancellationException | ExecutionException | InterruptedException e) {
      cooja.doRemoveSimulation(false);
      return null;
    }
  }

  /** Opens a file chooser if the file cannot be read. */
  private static File validateFileOrSelectNew(File file) {
    if (file != null && file.canRead()) {
      return file;
    }
    final File suggestedFile = file;
    return new Cooja.RunnableInEDT<File>() {
      @Override
      public File work() {
        JFileChooser fc = newFileChooser();
        if (suggestedFile != null && suggestedFile.isDirectory()) {
          fc.setCurrentDirectory(suggestedFile);
        } else {
          // Suggest file using file history.
          File suggestedFile = getLastOpenedFile();
          if (suggestedFile != null) {
            fc.setSelectedFile(suggestedFile);
          }
        }
        if (fc.showOpenDialog(frame) != JFileChooser.APPROVE_OPTION) {
          return null;
        }
        File file = fc.getSelectedFile();
        if (!file.exists()) {  // Try default file extension.
          file = new File(file.getParent(), file.getName() + fc.getFileFilter());
        }
        if (!file.exists() || !file.canRead()) {
          logger.fatal("No read access to file");
          return null;
        }
        return file;
      }
    }.invokeAndWait();
  }

  public File doSaveConfig() {
    cooja.getSimulation().stopSimulation();
    JFileChooser fc = newFileChooser();
    if (fc.showSaveDialog(myDesktopPane) != JFileChooser.APPROVE_OPTION) {
      return null;
    }
    File saveFile = fc.getSelectedFile();
    if (!fc.accept(saveFile)) {
      saveFile = new File(saveFile.getParent(), saveFile.getName() + fc.getFileFilter());
    }
    if (saveFile.exists()) {
      Object[] opts = {"Overwrite", "Cancel"};
      if (JOptionPane.showOptionDialog(frame, "A file with the same name already exists.\nDo you want to remove it?",
              "Overwrite existing file?", JOptionPane.YES_NO_OPTION,
              JOptionPane.QUESTION_MESSAGE, null, opts, opts[0]) != JOptionPane.YES_OPTION) {
        return null;
      }
    }
    if (saveFile.exists() && !saveFile.canWrite()) {
      JOptionPane.showMessageDialog(frame, "No write access to " + saveFile, "Save failed", JOptionPane.ERROR_MESSAGE);
      logger.fatal("No write access to file: " + saveFile.getAbsolutePath());
      return null;
    }
    cooja.saveSimulationConfig(saveFile);
    addToFileHistory(saveFile);
    return saveFile;
  }

  /**
   * Create a worker that will load the simulation in the background, displaying
   * a progress dialog if it is a quick-load.
   *
   * @param configFile Configuration file to load, reloads current sim if null
   * @param quick      Quick-load simulation
   * @param rewriteCsc Rewrite simulation config
   * @param manualRandomSeed The random seed to use for the simulation
   * @return The worker that will load the simulation.
   */
  public SwingWorker<Simulation, Cooja.SimulationCreationException> createLoadSimWorker(File configFile, final boolean quick,
                                                                                         boolean rewriteCsc, Long manualRandomSeed) {
    assert java.awt.EventQueue.isDispatchThread() : "Call from AWT thread";
    final var autoStart = configFile == null && cooja.getSimulation().isRunning();
    if (configFile != null && !cooja.doRemoveSimulation(!cooja.configuration.updateSim())) {
      return null;
    }

    if (configFile != null) {
      addToFileHistory(configFile);
    }

    final JPanel progressPanel;
    final JDialog progressDialog;
    if (quick) {
      final String progressTitle = "Loading " + (configFile == null ? "" : configFile.getAbsolutePath());
      progressDialog = new JDialog(frame, progressTitle, Dialog.ModalityType.APPLICATION_MODAL);

      progressPanel = new JPanel(new BorderLayout());
      var progressBar = new JProgressBar(0, 100);
      progressBar.setValue(0);
      progressBar.setIndeterminate(true);

      PROGRESS_BAR = progressBar; // Allow various parts of Cooja to show messages.

      var button = new JButton("Abort");

      progressPanel.add(BorderLayout.CENTER, progressBar);
      progressPanel.add(BorderLayout.SOUTH, button);
      progressPanel.setBorder(BorderFactory.createEmptyBorder(20, 20, 20, 20));

      progressDialog.getContentPane().add(progressPanel);
      progressDialog.setSize(400, 200);

      progressDialog.getRootPane().setDefaultButton(button);
      progressDialog.setLocationRelativeTo(frame);
      progressDialog.setDefaultCloseOperation(JDialog.DO_NOTHING_ON_CLOSE);
    } else {
      progressPanel = null;
      progressDialog = null;
    }

    final var cfgFile = configFile;
    // SwingWorker can pass information from worker to process() through publish().
    // Communicate information the other way through this shared queue.
    final var channel = new SynchronousQueue<Integer>(true);
    var worker = new SwingWorker<Simulation, Cooja.SimulationCreationException>() {
      @Override
      public Simulation doInBackground() {
        Element root = cfgFile == null ? cooja.extractSimulationConfig() : null;
        boolean shouldRetry;
        Simulation newSim = null;
        do {
          try {
            shouldRetry = false;
            PROGRESS_WARNINGS.clear();
            if (cfgFile == null) {
              cooja.doRemoveSimulation(false);
              newSim = cooja.createSimulation(root, quick, rewriteCsc, manualRandomSeed);
              cooja.setSimulation(newSim);
            } else {
              newSim = cooja.loadSimulationConfig(cfgFile, quick, rewriteCsc, manualRandomSeed);
            }
            if (newSim != null && autoStart) {
              newSim.startSimulation();
            }
          } catch (Cooja.SimulationCreationException e) {
            publish(e);
            try {
              shouldRetry = channel.take() == 1;
            } catch (InterruptedException ex) {
              cooja.doRemoveSimulation(false);
              return null;
            }
          }
        } while (shouldRetry);
        return newSim;
      }

      @Override
      protected void process(List<Cooja.SimulationCreationException> exs) {
        for (var e : exs) {
          var retry = showErrorDialog("Simulation load error", e, true);
          try {
            channel.put(retry ? 1 : 0);
          } catch (InterruptedException ex) {
            cancel(true);
            return;
          }
        }
      }

      @Override
      protected void done() {
        updateProgress(false);
        // Optionally show compilation warnings.
        var hideWarn = Boolean.parseBoolean(Cooja.getExternalToolsSetting("HIDE_WARNINGS", "false"));
        if (quick && !hideWarn && !PROGRESS_WARNINGS.isEmpty()) {
          final String[] warnings = PROGRESS_WARNINGS.toArray(new String[0]);
          final JDialog dialog = new JDialog(GUI.frame, "Compilation warnings", false);
          // Warnings message list.
          MessageListUI compilationOutput = new MessageListUI();
          for (String w : warnings) {
            compilationOutput.addMessage(w, MessageList.ERROR);
          }
          compilationOutput.addPopupMenuItem(null, true);
          Box buttonBox = Box.createHorizontalBox();
          buttonBox.add(Box.createHorizontalGlue());
          JCheckBox hideButton = new JCheckBox("Hide compilation warnings", false);
          hideButton.addActionListener(e ->
                  Cooja.setExternalToolsSetting("HIDE_WARNINGS", String.valueOf(((JCheckBox) e.getSource()).isSelected())));
          buttonBox.add(Box.createHorizontalStrut(10));
          buttonBox.add(hideButton);

          // Close on escape.
          InputMap inputMap = dialog.getRootPane().getInputMap(JComponent.WHEN_ANCESTOR_OF_FOCUSED_COMPONENT);
          inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0, false), "close");
          dialog.getRootPane().getActionMap().put("close", new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
              dialog.dispose();
            }
          });

          // Layout.
          dialog.getContentPane().add(BorderLayout.CENTER, new JScrollPane(compilationOutput));
          dialog.getContentPane().add(BorderLayout.SOUTH, buttonBox);
          dialog.setSize(700, 500);
          dialog.setLocationRelativeTo(frame);
          dialog.setVisible(true);
        }
        PROGRESS_WARNINGS.clear();
        if (progressDialog != null && progressDialog.isDisplayable()) {
          progressDialog.dispose();
        }
      }
    };

    if (progressDialog != null) {
      java.awt.EventQueue.invokeLater(() -> {
        progressPanel.setVisible(true);
        progressDialog.getRootPane().getDefaultButton().addActionListener(e -> worker.cancel(true));
        progressDialog.setVisible(true);
      });
    }
    return worker;
  }

  public void updateProgress(boolean stoppedSimulation) {
    java.awt.EventQueue.invokeLater(() -> toolbarListener.updateToolbar(stoppedSimulation));
  }

  public void updateGUIComponentState() {
    java.awt.EventQueue.invokeLater(() -> {
      // Update action state.
      for (var a : guiActions) {
        a.setEnabled(a.shouldBeEnabled());
      }

      // Mote and mote type menus.
      menuMoteTypeClasses.setEnabled(cooja.getSimulation() != null);
      menuMoteTypes.setEnabled(cooja.getSimulation() != null);
      updateProgress(false);
    });
  }

  private static void updateDesktopSize(final JDesktopPane desktop) {
    if (desktop == null || !desktop.isVisible() || desktop.getParent() == null) {
      return;
    }

    var rect = desktop.getVisibleRect();
    var pref = new Dimension(rect.width - 1, rect.height - 1);
    for (var frame : desktop.getAllFrames()) {
      if (pref.width < frame.getX() + frame.getWidth() - 20) {
        pref.width = frame.getX() + frame.getWidth();
      }
      if (pref.height < frame.getY() + frame.getHeight() - 20) {
        pref.height = frame.getY() + frame.getHeight();
      }
    }
    desktop.setPreferredSize(pref);
    desktop.revalidate();
  }

  public static void setProgressMessage(String msg, int type) {
    if (PROGRESS_BAR != null && PROGRESS_BAR.isShowing()) {
      PROGRESS_BAR.setString(msg);
      PROGRESS_BAR.setStringPainted(true);
    }
    if (type != MessageListUI.NORMAL) {
      PROGRESS_WARNINGS.add(msg);
    }
  }

  private static File getLastOpenedFile() {
    // Fetch current history
    String[] historyArray = Cooja.getExternalToolsSetting("SIMCFG_HISTORY", "").split(";");
    return historyArray.length > 0 ? new File(historyArray[0]) : null;
  }

  private static File[] getFileHistory() {
    // Fetch current history
    String[] historyArray = Cooja.getExternalToolsSetting("SIMCFG_HISTORY", "").split(";");
    File[] history = new File[historyArray.length];
    for (int i = 0; i < historyArray.length; i++) {
      history[i] = new File(historyArray[i]);
    }
    return history;
  }

  /** Adds a file first to the file history. Method avoids adding duplicates. */
  private void addToFileHistory(File file) {
    // Fetch current history
    String[] history = Cooja.getExternalToolsSetting("SIMCFG_HISTORY", "").split(";");
    String newFile;
    try {
      newFile = file.getCanonicalPath();
    } catch (IOException e) {
      return;
    }
    if (history.length > 0 && history[0].equals(newFile)) {
      // File already added
      return;
    }
    // Create new history
    StringBuilder newHistory = new StringBuilder();
    newHistory.append(newFile);
    for (int i = 0, count = 1; i < history.length && count < 10; i++) {
      String historyFile = history[i];
      if (!newFile.equals(historyFile) && historyFile.length() != 0) {
        newHistory.append(';').append(historyFile);
        count++;
      }
    }
    Cooja.setExternalToolsSetting("SIMCFG_HISTORY", newHistory.toString());
    Cooja.saveExternalToolsUserSettings();
    hasFileHistoryChanged = true;
  }

  /** Allocate a file chooser for Cooja simulation files. */
  private static JFileChooser newFileChooser() {
    JFileChooser fc = new JFileChooser();
    fc.setFileFilter(new FileFilter() {
      @Override
      public boolean accept(File file) {
        if (file.isDirectory()) {
          return true;
        }
        return file.getName().endsWith(".csc") || file.getName().endsWith(".csc.gz");
      }
      @Override
      public String getDescription() {
        return "Cooja simulation (.csc, .csc.gz)";
      }
      @Override
      public String toString() {
        return ".csc";
      }
    });

    // Suggest file using history
    File suggestedFile = getLastOpenedFile();
    if (suggestedFile != null) {
      fc.setSelectedFile(suggestedFile);
    }
    return fc;
  }

  public static boolean showErrorDialog(final String title, final Throwable e, final boolean retry) {
    return new Cooja.RunnableInEDT<Boolean>() {
      @Override
      public Boolean work() {
        JTabbedPane tabbedPane = new JTabbedPane();
        var buttonBox = Box.createHorizontalBox();
        // Contiki error.
        if (e instanceof ContikiError ex) {
          MessageListUI list = new MessageListUI();
          list.addMessage(e.getMessage());
          list.addMessage("");
          list.addMessage("");
          for (String l: ex.getContikiError().split("\n")) {
            list.addMessage(l);
          }
          list.addPopupMenuItem(null, true);
          tabbedPane.addTab("Contiki error", new JScrollPane(list));
        }
        // Compilation output.
        MessageListUI compilationOutput = null;
        if (e instanceof MoteType.MoteTypeCreationException ex && ex.hasCompilationOutput()) {
          compilationOutput = (MessageListUI) ex.getCompilationOutput();
        } else if (e != null && e.getCause() instanceof MoteType.MoteTypeCreationException ex && ex.hasCompilationOutput()) {
          compilationOutput = (MessageListUI) ex.getCompilationOutput();
        }
        if (compilationOutput != null) {
          compilationOutput.addPopupMenuItem(null, true);
          tabbedPane.addTab("Compilation output", new JScrollPane(compilationOutput));
        }
        if (e != null) {
          // Stack trace.
          MessageListUI stackTrace = new MessageListUI();
          e.printStackTrace(stackTrace.getInputStream(MessageListUI.NORMAL));
          stackTrace.addPopupMenuItem(null, true);
          tabbedPane.addTab("Java stack trace", new JScrollPane(stackTrace));

          // Exception message.
          buttonBox.add(Box.createHorizontalStrut(10));
          buttonBox.add(new JLabel(e.getMessage()));
          buttonBox.add(Box.createHorizontalStrut(10));
        }

        buttonBox.add(Box.createHorizontalGlue());
        final var dialog = new JDialog(frame, title, true);
        if (retry) {
          var retryAction = new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
              dialog.setTitle("-RETRY-");
              dialog.dispose();
            }
          };
          JButton retryButton = new JButton(retryAction);
          retryButton.setText("Retry Ctrl+R");
          buttonBox.add(retryButton);

          var inputMap = dialog.getRootPane().getInputMap(JComponent.WHEN_ANCESTOR_OF_FOCUSED_COMPONENT);
          inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_R, KeyEvent.CTRL_DOWN_MASK, false), "retry");
          dialog.getRootPane().getActionMap().put("retry", retryAction);
        }

        var closeAction = new AbstractAction() {
          @Override
          public void actionPerformed(ActionEvent e) {
            dialog.dispose();
          }
        };
        JButton closeButton = new JButton(closeAction);
        closeButton.setText("Close");
        buttonBox.add(closeButton);

        var inputMap = dialog.getRootPane().getInputMap(JComponent.WHEN_ANCESTOR_OF_FOCUSED_COMPONENT);
        inputMap.put(KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0, false), "close");
        dialog.getRootPane().getActionMap().put("close", closeAction);
        dialog.getRootPane().setDefaultButton(closeButton);
        dialog.getContentPane().add(BorderLayout.CENTER, tabbedPane);
        dialog.getContentPane().add(BorderLayout.SOUTH, buttonBox);
        dialog.setSize(700, 500);
        dialog.setLocationRelativeTo(frame);
        dialog.setVisible(true); // BLOCKS.
        return dialog.getTitle().equals("-RETRY-");
      }
    }.invokeAndWait();
  }

  private static boolean warnMemory() {
    long max = Runtime.getRuntime().maxMemory();
    long used = Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory();
    double memRatio = (double) used / (double) max;
    if (memRatio < 0.8) {
      return false;
    }

    DecimalFormat format = new DecimalFormat("0.000");
    logger.warn("Memory usage is getting critical. Reboot Cooja to avoid out of memory error. Current memory usage is " + format.format(100*memRatio) + "%.");
    int n = JOptionPane.showOptionDialog(frame,
            "Reboot Cooja to avoid out of memory error.\n" +
                    "Current memory usage is " + format.format(100 * memRatio) + "%.",
            "Out of memory warning", JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE, null,
            new String[]{"Continue", "Abort"}, "Abort");
    return n != JOptionPane.YES_OPTION;
  }

  public static void setLookAndFeel() throws InterruptedException, InvocationTargetException {
    java.awt.EventQueue.invokeAndWait(() -> {
      JFrame.setDefaultLookAndFeelDecorated(true);
      JDialog.setDefaultLookAndFeelDecorated(true);
      ToolTipManager.sharedInstance().setDismissDelay(60000);
      // Nimbus.
      try {
        String osName = System.getProperty("os.name").toLowerCase();
        if (osName.startsWith("linux")) {
          try {
            for (var info : UIManager.getInstalledLookAndFeels()) {
              if ("Nimbus".equals(info.getName())) {
                UIManager.setLookAndFeel(info.getClassName());
                break;
              }
            }
          } catch (UnsupportedLookAndFeelException e) {
            UIManager.setLookAndFeel(UIManager.getCrossPlatformLookAndFeelClassName());
          }
        } else {
          UIManager.setLookAndFeel("javax.swing.plaf.nimbus.NimbusLookAndFeel");
        }
        return;
      } catch (Exception e) {
      }

      // System.
      try {
        UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
      } catch (Exception e) {
      }
    });
  }

  /** GUI event handler */
  private class GUIEventHandler implements ActionListener {
    @Override
    public void actionPerformed(ActionEvent e) {
      MoteType newMoteType = null;
      final var cmd = e.getActionCommand();
      switch (cmd) {
        case "create mote type" -> {
          cooja.getSimulation().stopSimulation();
          // Create mote type
          var clazz = (Class<? extends MoteType>) ((JMenuItem) e.getSource()).getClientProperty("class");
          try {
            newMoteType = MoteInterfaceHandler.createMoteType(cooja, clazz.getName());
            if (newMoteType == null) {
              newMoteType = clazz.getDeclaredConstructor().newInstance();
            }
            if (!newMoteType.configureAndInit(frame, cooja.getSimulation(), true)) {
              return;
            }
            cooja.getSimulation().addMoteType(newMoteType);
          } catch (Exception e1) {
            logger.fatal("Exception when creating mote type", e1);
            showErrorDialog("Mote type creation error", e1, false);
            newMoteType = null;
          }
        }
        case "add motes" -> {
          cooja.getSimulation().stopSimulation();
          newMoteType = (MoteType) ((JMenuItem) e.getSource()).getClientProperty("motetype");
        }
        case "edit paths" -> ExternalToolsDialog.showDialog();
        case "manage extensions" -> {
          COOJAProject[] newProjects = ProjectDirectoriesDialog.showDialog(cooja, cooja.getProjects());
          if (newProjects != null) {
            cooja.currentProjects.clear();
            cooja.currentProjects.addAll(Arrays.asList(newProjects));
            try {
              cooja.reparseProjectConfig();
            } catch (Cooja.ParseProjectsException ex) {
              logger.fatal("Error when loading extensions: " + ex.getMessage(), ex);
              JOptionPane.showMessageDialog(frame,
                      "All Cooja extensions could not load.\n\n" +
                              "To manage Cooja extensions:\n" +
                              "Menu->Settings->Cooja extensions",
                      "Reconfigure Cooja extensions", JOptionPane.INFORMATION_MESSAGE);
              showErrorDialog("Cooja extensions load error", ex, false);
            }
          }
        }
        default -> logger.warn("Unhandled action: " + cmd);
      }
      if (newMoteType != null) {
        for (var mote : AddMoteDialog.showDialog(cooja.getSimulation(), newMoteType)) {
          cooja.getSimulation().addMote(mote);
        }
      }
    }
  }

  private interface ToolbarListener extends ItemListener {
    /** Updates buttons according to simulation status. */
    void updateToolbar(boolean stoppedSimulation);
  }

  /** GUI actions */
  abstract static class GUIAction extends AbstractAction {
    public GUIAction(String name) {
      super(name);
    }
    public GUIAction(String name, int mnemonic) {
      this(name);
      putValue(Action.MNEMONIC_KEY, mnemonic);
    }
    public GUIAction(String name, KeyStroke accelerator) {
      this(name);
      putValue(Action.ACCELERATOR_KEY, accelerator);
    }
    public GUIAction(String name, int mnemonic, KeyStroke accelerator) {
      this(name, mnemonic);
      putValue(Action.ACCELERATOR_KEY, accelerator);
    }
    public abstract boolean shouldBeEnabled();
  }
  class StartPluginGUIAction extends GUIAction {
    public StartPluginGUIAction(String name) {
      super(name);
    }
    @Override
    public void actionPerformed(final ActionEvent e) {
      new Thread(() -> {
        var pluginClass = (Class<Plugin>) ((JMenuItem) e.getSource()).getClientProperty("class");
        Mote mote = (Mote) ((JMenuItem) e.getSource()).getClientProperty("mote");
        cooja.tryStartPlugin(pluginClass, cooja.getSimulation(), mote);
      }, "StartPluginGUIAction").start();
    }
    @Override
    public boolean shouldBeEnabled() {
      return cooja.getSimulation() != null;
    }
  }
}
