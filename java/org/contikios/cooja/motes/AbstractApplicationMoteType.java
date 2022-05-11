/*
 * Copyright (c) 2007, Swedish Institute of Computer Science. All rights
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

package org.contikios.cooja.motes;

import java.awt.Container;
import java.io.File;
import java.util.ArrayList;
import java.util.Collection;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.LogManager;
import org.jdom.Element;

import org.contikios.cooja.ClassDescription;
import org.contikios.cooja.Cooja;
import org.contikios.cooja.Mote;
import org.contikios.cooja.MoteInterface;
import org.contikios.cooja.MoteType;
import org.contikios.cooja.ProjectConfig;
import org.contikios.cooja.Simulation;
import org.contikios.cooja.interfaces.ApplicationLED;
import org.contikios.cooja.interfaces.ApplicationRadio;
import org.contikios.cooja.interfaces.ApplicationSerialPort;
import org.contikios.cooja.interfaces.Mote2MoteRelations;
import org.contikios.cooja.interfaces.MoteAttributes;
import org.contikios.cooja.interfaces.MoteID;
import org.contikios.cooja.interfaces.Position;

@ClassDescription("Application Mote Type")
public abstract class AbstractApplicationMoteType implements MoteType {
  private static final Logger logger = LogManager.getLogger(AbstractApplicationMoteType.class);

  private ProjectConfig myConfig = null;

  private String identifier = null;
  private String description = null;

  @SuppressWarnings("unchecked")
  private final Class<? extends MoteInterface>[] moteInterfaceClasses = new Class[] {
      SimpleMoteID.class,
      Position.class,
      ApplicationSerialPort.class,
      ApplicationRadio.class,
      ApplicationLED.class,
      Mote2MoteRelations.class,
      MoteAttributes.class
  };

  public AbstractApplicationMoteType() {
    super();
  }

  public AbstractApplicationMoteType(String identifier) {
    super();
    this.identifier = identifier;
    this.description = "Application Mote Type #" + identifier;
  }

  @Override
  public boolean configureAndInit(Container parentContainer, Simulation simulation, boolean visAvailable)
  throws MoteTypeCreationException {
    if (identifier == null) {
      /* Create unique identifier */
      int counter = 0;
      boolean identifierOK = false;
      while (!identifierOK) {
        counter++;
        identifier = "apptype" + counter;
        identifierOK = true;

        // Check if identifier is already used by some other type
        for (MoteType existingMoteType : simulation.getMoteTypes()) {
          if (existingMoteType != this
              && existingMoteType.getIdentifier().equals(identifier)) {
            identifierOK = false;
            break;
          }
        }
      }
    }
    if (description == null) {
      description = "Application Mote Type #" + identifier;
    }
    return true;
  }

  @Override
  public String getIdentifier() {
    return identifier;
  }

  @Override
  public void setIdentifier(String identifier) {
    this.identifier = identifier;
  }

  @Override
  public String getDescription() {
    return description;
  }

  @Override
  public void setDescription(String description) {
    this.description = description;
  }

  @Override
  public Class<? extends MoteInterface>[] getMoteInterfaceClasses() {
    return moteInterfaceClasses;
  }

  @Override
  public void setMoteInterfaceClasses(Class<? extends MoteInterface>[] moteInterfaces) {
    throw new RuntimeException("Can not change the mote interface classes");
  }

  @Override
  public JComponent getTypeVisualizer() {
    StringBuilder sb = new StringBuilder();
    // Identifier
    sb.append("<html><table><tr><td>Identifier</td><td>")
    .append(getIdentifier()).append("</td></tr>");

    // Description
    sb.append("<tr><td>Description</td><td>")
    .append(getDescription()).append("</td></tr>");

    sb.append("<tr><td valign=\"top\">Mote interface</td><td>");
    for (Class<? extends MoteInterface> moteInterface : moteInterfaceClasses) {
      sb.append(moteInterface.getSimpleName()).append("<br>");
    }
    sb.append("</td></tr>");

    JLabel label = new JLabel(sb.append("</table></html>").toString());
    label.setVerticalTextPosition(JLabel.TOP);
    return label;
  }

  @Override
  public File getContikiSourceFile() {
    return null; /* Contiki-independent */
  }

  @Override
  public File getContikiFirmwareFile() {
    return null; /* Contiki-independent */
  }

  @Override
  public void setContikiSourceFile(File file) {
    /* Contiki-independent */
  }

  @Override
  public void setContikiFirmwareFile(File file) {
    /* Contiki-independent */
  }

  @Override
  public String getCompileCommands() {
    /* Contiki-independent */
    return null;
  }

  @Override
  public void setCompileCommands(String commands) {
    /* Contiki-independent */
  }

  @Override
  public ProjectConfig getConfig() {
    return myConfig;
  }

  @Override
  public Collection<Element> getConfigXML(Simulation simulation) {
    ArrayList<Element> config = new ArrayList<Element>();
    Element element;

    element = new Element("identifier");
    element.setText(getIdentifier());
    config.add(element);

    element = new Element("description");
    element.setText(getDescription());
    config.add(element);

    return config;
  }

  @Override
  public boolean setConfigXML(Simulation simulation,
      Collection<Element> configXML, boolean visAvailable)
  throws MoteTypeCreationException {
    for (Element element : configXML) {
      String name = element.getName();
      if (name.equals("identifier")) {
        identifier = element.getText();
      } else if (name.equals("description")) {
        description = element.getText();
      }
    }

    boolean createdOK = configureAndInit(Cooja.getTopParentContainer(), simulation, visAvailable);
    return createdOK;
  }

  public static class SimpleMoteID extends MoteID {
    private int id = -1;
    public SimpleMoteID(Mote mote) {
    }
    @Override
    public int getMoteID() {
      return id;
    }
    @Override
    public void setMoteID(int newID) {
      this.id = newID;
    }
    @Override
    public JPanel getInterfaceVisualizer() {
      return null;
    }
    @Override
    public void releaseInterfaceVisualizer(JPanel panel) {
    }
  }

}
