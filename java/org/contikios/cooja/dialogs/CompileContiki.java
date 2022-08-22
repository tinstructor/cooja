/*
 * Copyright (c) 2009, Swedish Institute of Computer Science.
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

package org.contikios.cooja.dialogs;

import static java.nio.charset.StandardCharsets.UTF_8;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.swing.Action;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.contikios.cooja.MoteType.MoteTypeCreationException;

/**
 * Contiki compiler library.
 * Uses notion of Contiki platforms to compile a Contiki firmware.
 *
 * @author Fredrik Osterlind
 */
public class CompileContiki {
  private static final Logger logger = LogManager.getLogger(CompileContiki.class);

  /**
   * Executes a Contiki compilation command.
   *
   * @param commandIn Command
   * @param env (Optional) Environment. May be null.
   * @param directory Directory in which to execute command
   * @param onSuccess Action called if compilation succeeds
   * @param onFailure Action called if compilation fails
   * @param compilationOutput Is written both std and err process output
   * @param synchronous If true, method blocks until process completes
   * @return Sub-process if called asynchronously
   * @throws Exception If process returns error, or outputFile does not exist
   */
  public static Process compile(
      final String commandIn,
      final String[] env,
      final File directory,
      final Action onSuccess,
      final Action onFailure,
      final MessageList compilationOutput,
      boolean synchronous)
  throws Exception {
    Pattern p = Pattern.compile("([^\\s\"']+|\"[^\"]*\"|'[^']*')");
    Matcher m = p.matcher(commandIn);
    ArrayList<String> commandList = new ArrayList<>();
    while(m.find()) {
      String arg = m.group();
      if (arg.length() > 1 && (arg.charAt(0) == '"' || arg.charAt(0) == '\'')) {
          arg = arg.substring(1, arg.length() - 1);
      }
      commandList.add(arg);
    }

  	/* TODO Fix me */
    final MessageList messageDialog =
        Objects.requireNonNullElseGet(compilationOutput, () -> MessageContainer.createMessageList(true));
    String cpus = Integer.toString(Runtime.getRuntime().availableProcessors());
    // Perform compile command variable expansions.
    String[] command = new String[commandList.size()];
    for (int i = 0; i < commandList.size(); i++) {
      command[i] = commandList.get(i).replace("$(CPUS)", cpus);
    }
    {
      var cmd = new StringBuilder();
      for (String c: command) {
      	cmd.append(c).append(" ");
      }
      messageDialog.addMessage("", MessageList.NORMAL);
      messageDialog.addMessage("> " + cmd, MessageList.NORMAL);
    }

    final Process compileProcess;
    try {
      compileProcess = Runtime.getRuntime().exec(command, env, directory);

      Thread readInput = new Thread(new Runnable() {
        @Override
        public void run() {
          try (var stdout = new BufferedReader(new InputStreamReader(compileProcess.getInputStream(), UTF_8))) {
            String readLine;
            while ((readLine = stdout.readLine()) != null) {
              messageDialog.addMessage(readLine, MessageList.NORMAL);
            }
          } catch (IOException e) {
            logger.warn("Error while reading from process");
          }
        }
      }, "read input stream thread");

      Thread readError = new Thread(new Runnable() {
        @Override
        public void run() {
          try (var stderr = new BufferedReader(new InputStreamReader(compileProcess.getErrorStream(), UTF_8))) {
            String readLine;
            while ((readLine = stderr.readLine()) != null) {
              messageDialog.addMessage(readLine, MessageList.ERROR);
            }
          } catch (IOException e) {
            logger.warn("Error while reading from process");
          }
        }
      }, "read error stream thread");

      final MoteTypeCreationException syncException = new MoteTypeCreationException("");
      Thread handleCompilationResultThread = new Thread(new Runnable() {
        @Override
        public void run() {

          /* Wait for compilation to end */
          try {
            compileProcess.waitFor();
          } catch (Exception e) {
            messageDialog.addMessage(e.getMessage(), MessageList.ERROR);
            syncException.setCompilationOutput(MessageContainer.createMessageList(true));
            syncException.fillInStackTrace();
            return;
          }

          /* Check return value */
          if (compileProcess.exitValue() != 0) {
            messageDialog.addMessage("Process returned error code " + compileProcess.exitValue(), MessageList.ERROR);
            if (onFailure != null) {
              onFailure.actionPerformed(null);
            }
            syncException.setCompilationOutput(MessageContainer.createMessageList(true));
            syncException.fillInStackTrace();
            return;
          }

          if (onSuccess != null) {
            onSuccess.actionPerformed(null);
          }
        }
      }, "handle compilation results");

      readInput.start();
      readError.start();
      handleCompilationResultThread.start();

      if (synchronous) {
        try {
          handleCompilationResultThread.join();
        } catch (Exception e) {
          /* Make sure process has exited */
          compileProcess.destroy();

          String msg = e.getMessage();
          if (e instanceof InterruptedException) {
            msg = "Aborted by user";
          }
          throw new MoteTypeCreationException("Compilation error: " + msg, e);
        }

        /* Detect error manually */
        if (syncException.hasCompilationOutput()) {
          throw new MoteTypeCreationException("Bad return value", syncException);
        }
      }
    } catch (IOException ex) {
      if (onFailure != null) {
        onFailure.actionPerformed(null);
      }
      throw new MoteTypeCreationException("Compilation error: " + ex.getMessage(), ex);
    }

    return compileProcess;
  }
}
