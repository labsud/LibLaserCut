/**
 * This file is part of LibLaserCut.
 * Copyright (C) 2011 - 2014 Thomas Oster <mail@thomas-oster.de>
 *
 * LibLaserCut is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LibLaserCut is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with LibLaserCut. If not, see <http://www.gnu.org/licenses/>.
 *
 **/
package com.t_oster.liblasercut.drivers;

import com.t_oster.liblasercut.*;
import com.t_oster.liblasercut.platform.Util;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import purejavacomm.*;
import java.util.*;

/**
 * This class implements a generic driver for GRBL based lasercuters.
 * 
 */
public class GrblLaser extends LaserCutter {

  private static final String SETTING_COMPORT = "COM Port";
  private static final String SETTING_BEDWIDTH = "Laserbed width";
  private static final String SETTING_BEDHEIGHT = "Laserbed height";
  private static final String SETTING_MAX_SPEED = "Max speed rate (in mm/min)";
  private static final String SETTING_MAX_FEED = "Max feed rate (in mm/min)";
  private static final String SETTING_PRE_JOB_GCODE = "Pre-Job GCode (comma separated)";
  private static final String SETTING_POST_JOB_GCODE = "Post-Job GCode (comma separated)";
  private static final String SETTING_IDENTIFICATION_LINE = "Banner / Identification starts with";
  private static final String SETTING_INIT_DELAY = "Sec to wait for board reset. 0 for soft reset.";
  private static final String LINEEND = "\n";
  private static final String SETTING_HOMING = "Do homing ($H) on init (y/n)";  
  private static final String SETTING_COMSPEED = "COM Speed";  
  
  
  // Working in mm, Absolute distance mode, set origin to current
  // if homing is activated, current is machine home, after setting origin to current,
  // machine cords will be work cords.
  // Since there's no possinility to jog inside Visicut, laser cutter should be equipped with homing switches,
  // homing should be activated in GRBL ($22=1) and in visicut (SETTING_HOMING = true)
  // For visicut, homing origin have to be in the machine uper left corner.
  protected String preJobGcode = "G21,G90,G10 P0 L20 X0,G10 L20 Y0";
  

  public String getPreJobGcode()
  {
    return preJobGcode;
  }

  public void setPreJobGcode(String preJobGcode)
  {
    this.preJobGcode = preJobGcode;
  }
  
  protected String postJobGcode = "G0 X0 Y0";

  public String getPostJobGcode()
  {
    return postJobGcode;
  }

  public void setPostJobGcode(String postJobGcode)
  {
    this.postJobGcode = postJobGcode;
  }
  
  /**
   * What is expected to be received after serial/telnet connection
   * 
   */
  protected String identificationLine = "Grbl";

  public String getIdentificationLine()
  {
    return identificationLine;
  }
  
  public void setIdentificationLine(String identificationLine)
  {
    this.identificationLine = identificationLine;
  }

   /**
   * Time to wait before firsts reads of serial port.
   * See autoreset feature on arduinos.
   */
  protected String initDelay = "5"; 

  public String getInitDelay()
  {
    return initDelay;
  }

  public void setInitDelay(String initDelay)
  {
    this.initDelay = initDelay;
  }
  
  @Override
  public String getModelName() {
    return "Grbl Laser";
  }
  
  /**
   * Com port where GRBL is connected to
   * Set to auto for auto detection
   */
  protected String comport = "/dev/tty.usbmodem1411";

  public String getComport()
  {
    return comport;
  }

  public void setComport(String comport)
  {
    this.comport = comport;
  }

  protected String comspeed = "115200";

  public String getComspeed()
  {
    return comspeed;
  }
  
  public void setComspeed(String comport)
  {
    this.comspeed = comspeed;
  }

  protected boolean homing = false;

  public boolean getHoming()
  {
    return homing;
  }
  
  public void setHoming(boolean homing)
  {
    this.homing = homing;
  }
  
  protected double max_speed = 20*60;

  public double getMax_speed()
  {
    return max_speed;
  }

  public void setMax_speed(double max_speed)
  {
    this.max_speed = max_speed;
  }

  protected double max_feed = 100*60;

  public double getMax_feed()
  {
    return max_feed;
  }

  public void setMax_feed(double max_feed)
  {
    this.max_feed = max_feed;
  }
  
  @Override
  /**
   * We do not support Frequency atm, so we return power,speed and focus
   */
  public LaserProperty getLaserPropertyForVectorPart() {
    return new PowerSpeedFocusProperty();
  }

  protected void writeVectorGCode(VectorPart vp, double resolution) throws UnsupportedEncodingException, IOException {
    for (VectorCommand cmd : vp.getCommandList()) {
      switch (cmd.getType()) {
        case MOVETO:
          int x = cmd.getX();
          int y = cmd.getY();
          move(out, x, y, resolution);
          break;
        case LINETO:
          x = cmd.getX();
          y = cmd.getY();
          line(out, x, y, resolution);
          break;
        case SETPROPERTY:
          PowerSpeedFocusProperty p = (PowerSpeedFocusProperty) cmd.getProperty();
          setPower(p.getPower());
          setSpeed(p.getSpeed());
          setFocus(out, p.getFocus(), resolution);
          break;
      }
    }
  }
  private double currentPower = -1;
  private double currentSpeed = -1;
  private double nextPower = -1;
  private double nextSpeed = -1;
  private double currentFocus = 0;
  private boolean laserPowerSuspended = false;

  protected void setSpeed(double speedInPercent) {
    nextSpeed = speedInPercent;
  }

  protected void setPower(double powerInPercent) {
    nextPower = powerInPercent;
  }
  
  protected void setFocus(PrintStream out, double focus, double resolution) throws IOException {
    String append = "";
    if (!laserPowerSuspended) 
    {
      laserPowerSuspended = true;
      append += " S0";
    } 
    if (currentFocus != focus)
    {
      sendLine("G0 Z%f"+append, Util.px2mm(focus, resolution));
      currentFocus = focus;
    }
  }

  protected void move(PrintStream out, int x, int y, double resolution) throws IOException {
    String append = "";
    if (!laserPowerSuspended) 
    {
      laserPowerSuspended = true;
      sendLine ("M5");
      append += " S0";
      append += String.format(Locale.US, " F%d", (int) max_speed);

    } 
    sendLine("G0 X%f Y%f"+append, Util.px2mm(x, resolution), Util.px2mm(y, resolution));
  }

  protected void line(PrintStream out, int x, int y, double resolution) throws IOException {
    String append = "";
    if (laserPowerSuspended) {
      // reenable laser
      sendLine("M3");
      // and send current feed rate
      append += String.format(Locale.US, " F%d", (int) currentSpeed);
      }
    if ((nextPower != currentPower) || (laserPowerSuspended))
    {
      laserPowerSuspended=false;
      append += String.format(Locale.US, " S%f", nextPower);
      currentPower = nextPower;
    }
    if (nextSpeed != currentSpeed)
    {
      append += String.format(Locale.US, " F%d", (int) (max_speed*nextSpeed/100.0));
      currentSpeed = nextSpeed;
    }
    sendLine("G1 X%f Y%f"+append, Util.px2mm(x, resolution), Util.px2mm(y, resolution));
  }

  private void writeInitializationCode() throws IOException {
    if (homing == true)
    {
      // ['$H'|'$X' to unlock]
      System.out.println("Homing...");
      sendLine("$H");
    }   

    if (preJobGcode != null)
    {
      for (String line : preJobGcode.split(","))
      {
        sendLine(line);
      }
    }
  }

  private void writeShutdownCode() throws IOException {
    if (postJobGcode != null)
    {
      for (String line : postJobGcode.split(","))
      {
        sendLine(line);
      }
    }
  }

  private BufferedReader in;
  private PrintStream out;
  private CommPort port;
  private CommPortIdentifier portIdentifier;
  private boolean waitForOKafterEachLine = true;
  
  protected void sendLine(String text, Object... parameters) throws IOException
  {
    // write command to serial port
    out.printf(Locale.US,text+LINEEND, parameters);
    // debug gcode on system output
    System.out.printf(Locale.US,text+"; ", parameters);
    if (waitForOKafterEachLine)
    {
      String line = null;
      line=in.readLine();
      // debug grbl answer
      System.out.printf(Locale.US,"%s"+LINEEND, line);
      if (!"ok".equals(line))
      {
        disconnect();
        throw new IOException("Grbl did not answered 'ok'.\nAnswered: "+line);
      }
    }
  }
  
  protected String connect_serial(CommPortIdentifier i, ProgressListener pl) throws PortInUseException, IOException,  NoSuchPortException,  Exception
  {
    String line = null;
    pl.taskChanged(this, i.getName()); // opening
    if (i.getPortType() == CommPortIdentifier.PORT_SERIAL)
    {
      try
      {
        port = i.open("VisiCut", 1000);
        // Set serial port params
        SerialPort portopts = (SerialPort) port;
        portopts.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
        portopts.setDTR(true); // needed for Arduino Leonardo and Micro.
        portopts.setRTS(true); // needed for Arduino Leonardo and Micro
        portopts.setSerialPortParams(Integer.parseInt(comspeed), SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
        // open in & out
        out = new PrintStream(port.getOutputStream(), true, "US-ASCII");
        in = new BufferedReader(new InputStreamReader(port.getInputStream()));
        
        if (identificationLine != null && identificationLine.length() > 0)
        {
          // if initDelay set wait for it, this is and arduino who takes times to reset
          if (Integer.valueOf(initDelay)>0) {
            for (int openingDelay=0;openingDelay<Integer.valueOf(initDelay); openingDelay++) {
              pl.taskChanged(this, String.format("Waiting %ds", Integer.valueOf(initDelay)-openingDelay));
              try {
                    Thread.sleep(1000);                 //1000 milliseconds is one second.
                } catch(InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
            }
          }
          // Else, this is not an arduino.. Soft reset it.
          else {
            System.out.println("Soft reset on "+i.getName());
            out.printf(Locale.US,"\u0018");
            }
          pl.taskChanged(this, "opening '"+i.getName()+"'");
          int tryc;
          // try 3 times, since 1st line can be garbage after port opening
          // Search for :
          // Grbl 0.9i ['$' for help]
          for (tryc=0;tryc<3; tryc++)   
           {
             try {
               line= in.readLine();
              }
            catch (Exception e)
              {
              System.out.println("Line read: "+line);
              }
            if (line.startsWith(identificationLine))
              {
              System.out.println("Found board on "+i.getName());
              if (homing) {
                line= in.readLine();
                }
              break;
              }
            } 
           if (tryc == 3) // did not see id
           {
             in.close();
             out.close();
             port.close();
             return ("Does not seem to be a Grbl board on "+i.getName()+"\nAnwsered: "+line);
           }             
        }      
        portIdentifier = i;
        return null;
      }
      catch (PortInUseException e)
      {
        return "Port in use "+i.getName();
      }
      catch (IOException e)
      {
        return "IO Error "+i.getName();
      }
      catch (PureJavaIllegalStateException e)
      {
        return "Could not open "+i.getName();
      }
    }
    else
    {
      return "Not a serial Port "+comport;
    }
  }
  
  protected void connect(ProgressListener pl) throws IOException, PortInUseException, NoSuchPortException, Exception
  {
    if (comport != null && !comport.equals(""))
    {
      String error = "No serial port found";
      if (portIdentifier == null && !comport.equals("auto"))
      {
        try {
        portIdentifier = CommPortIdentifier.getPortIdentifier(comport);
        }
        catch (Exception e) {
         throw new Exception("Error: Could not Open serial '"+comport+"'");
         }

      }
      
      if (portIdentifier != null)
      {//use port identifier we had last time
        error = connect_serial(portIdentifier, pl);
      }
      else
      {
        Enumeration<CommPortIdentifier> e = CommPortIdentifier.getPortIdentifiers();
        while (e.hasMoreElements())
        {
          CommPortIdentifier i = e.nextElement();
          if (i.getPortType() == CommPortIdentifier.PORT_SERIAL)
          {
            System.out.println("Auto, testing port: "+i.getName());
            error = connect_serial(i, pl);
            if (error == null)
            {
              break;
            }
          }
        }
      }
      if (error != null)
      {
        throw new IOException(error);
      }
    }
  }
  
  protected void disconnect() throws IOException
  {
    in.close();
    out.close();
    if (this.port != null)
    {
      this.port.close();
      this.port = null;
    }
  }
  
  @Override
  public void sendJob(LaserJob job, ProgressListener pl, List<String> warnings) throws IllegalJobException, Exception {
    pl.progressChanged(this, 0);
    this.currentPower = -1;
    this.currentSpeed = -1;
    
    Runtime.getRuntime().addShutdownHook(new Thread() {
        @Override
        public void run() {
          SerialPort portopts = (SerialPort) port;
          portopts.setDTR(false); // Activate reset on arduino
          // Sleep 500ms
          try {
              Thread.sleep(500); 
          } catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
          }
          portopts.setDTR(true); // release reset         
        }
    });

    pl.taskChanged(this, "checking job");
    checkJob(job);
    job.applyStartPoint();
    pl.taskChanged(this, "connecting...");
    connect(pl);
    pl.taskChanged(this, "sending");
    writeInitializationCode();
    pl.progressChanged(this, 20);
    int i = 0;
    int max = job.getParts().size();
    for (JobPart p : job.getParts())
    {
      if (p instanceof RasterPart)
      {
        RasterPart rp = (RasterPart) p;
        LaserProperty black = rp.getLaserProperty();
        LaserProperty white = black.clone();
        white.setProperty("power", 0);
        p = convertRasterToVectorPart((RasterPart) p, black, white,  p.getDPI(), false);
      }
      if (p instanceof VectorPart)
      {
        //TODO: in direct mode use progress listener to indicate progress 
        //of individual job
        writeVectorGCode((VectorPart) p, p.getDPI());
      }
      i++;
      pl.progressChanged(this, 20 + (int) (i*(double) 60/max));
    }
    sendLine("M5");
    writeShutdownCode();
    disconnect();
    pl.taskChanged(this, "sent.");
    pl.progressChanged(this, 100);
  }
  private List<Double> resolutions;

  @Override
  public List<Double> getResolutions() {
    if (resolutions == null) {
      resolutions = Arrays.asList(new Double[]{
                500d
              });
    }
    return resolutions;
  }
  protected double bedWidth = 300;

  /**
   * Get the value of bedWidth
   *
   * @return the value of bedWidth
   */
  @Override
  public double getBedWidth() {
    return bedWidth;
  }

  /**
   * Set the value of bedWidth
   *
   * @param bedWidth new value of bedWidth
   */
  public void setBedWidth(double bedWidth) {
    this.bedWidth = bedWidth;
  }
  protected double bedHeight = 300;

  /**
   * Get the value of bedHeight
   *
   * @return the value of bedHeight
   */
  @Override
  public double getBedHeight() {
    return bedHeight;
  }

  /**
   * Set the value of bedHeight
   *
   * @param bedHeight new value of bedHeight
   */
  public void setBedHeight(double bedHeight) {
    this.bedHeight = bedHeight;
  }
  private static String[] settingAttributes = new String[]{
    SETTING_BEDWIDTH,
    SETTING_BEDHEIGHT,
    SETTING_COMPORT,
    SETTING_MAX_SPEED,
    SETTING_MAX_FEED,
    SETTING_PRE_JOB_GCODE,
    SETTING_POST_JOB_GCODE,
    SETTING_IDENTIFICATION_LINE,
    SETTING_INIT_DELAY,
    SETTING_COMSPEED,
    SETTING_HOMING
  };

  @Override
  public String[] getPropertyKeys() {
    return settingAttributes;
  }

  @Override
  public Object getProperty(String attribute) {
    if (SETTING_BEDWIDTH.equals(attribute)) {
      return this.getBedWidth();
    } else if (SETTING_BEDHEIGHT.equals(attribute)) {
      return this.getBedHeight();
    } else if (SETTING_COMPORT.equals(attribute)) {
      return this.getComport();
    } else if (SETTING_MAX_SPEED.equals(attribute)) {
      return this.getMax_speed();
    } else if (SETTING_MAX_FEED.equals(attribute)) {
      return this.getMax_feed();
    } else if (SETTING_PRE_JOB_GCODE.equals(attribute)) {
      return this.getPreJobGcode();
    } else if (SETTING_POST_JOB_GCODE.equals(attribute)) {
      return this.getPostJobGcode();
    } else if (SETTING_IDENTIFICATION_LINE.equals(attribute)) {
      return this.getIdentificationLine();
    } else if (SETTING_INIT_DELAY.equals(attribute)) {
      return this.getInitDelay();
    } else if (SETTING_COMSPEED.equals(attribute)) {
      return this.getComspeed();
    } else if (SETTING_HOMING.equals(attribute)) {
      return this.getHoming();
    }
    
    return null;
  }

  @Override
  public void setProperty(String attribute, Object value) {
    if (SETTING_BEDWIDTH.equals(attribute)) {
      this.setBedWidth((Double) value);
    } else if (SETTING_BEDHEIGHT.equals(attribute)) {
      this.setBedHeight((Double) value);
    } else if (SETTING_COMPORT.equals(attribute)) {
      this.setComport((String) value);
    } else if (SETTING_MAX_SPEED.equals(attribute)) {
      this.setMax_speed((Double) max_speed);
    } else if (SETTING_MAX_FEED.equals(attribute)) {
      this.setMax_feed((Double) max_feed);
    } else if (SETTING_PRE_JOB_GCODE.equals(attribute)) {
      this.setPreJobGcode((String) value);
    } else if (SETTING_POST_JOB_GCODE.equals(attribute)) {
      this.setPostJobGcode((String) value);
    } else if (SETTING_IDENTIFICATION_LINE.equals(attribute)) {
      this.setIdentificationLine((String) value);
    } else if (SETTING_INIT_DELAY.equals(attribute)) {
      this.setInitDelay((String) value);
    } else if (SETTING_COMSPEED.equals(attribute)) {
      this.setComspeed((String) value);
    } else if (SETTING_HOMING.equals(attribute)) {
      this.setHoming((Boolean) value);
    }
  }

  @Override
  public LaserCutter clone() {
    GrblLaser clone = new GrblLaser();
    clone.bedHeight = bedHeight;
    clone.bedWidth = bedWidth;
    clone.comport = comport;
    clone.max_speed = max_speed;
    clone.max_feed = max_feed;
    clone.preJobGcode = preJobGcode;
    clone.postJobGcode = postJobGcode;
    clone.identificationLine = identificationLine;
    clone.initDelay = initDelay;
    clone.comspeed = comspeed;
    clone.homing = homing;
    return clone;
  }
}
