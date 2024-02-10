package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.net.Socket;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.ProtocolException;
import java.net.UnknownHostException;
import java.io.IOException;
 
/**
 * Limelight camera sensor for FRC Romi
 */ 

public class RomiLimelight {
  public static final int LimelightNTRegistrationPort = 5899;

  protected NetworkTable m_table;
  private NetworkTableEntry m_tx, m_ty, m_ta, m_pipeline, m_ledMode;

  public double getX() { return m_tx.getDouble(0.0); }
  public double getY() { return m_ty.getDouble(0.0); }
  public double getA() { return m_ta.getDouble(0.0); }

  public int getPipeline() { return (int)m_pipeline.getDouble(-1); }
  public void setPipeline(int pipeline) { m_pipeline.setDouble(pipeline); }

  public void setLEDOn(boolean on) {
    if (on)
      m_ledMode.setNumber(3);
    else
      m_ledMode.setNumber(1);
  }

  /** Create a new RomiLimelight. */
  public RomiLimelight() {
    registerForNetworkTables();

    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_pipeline = m_table.getEntry("pipeline");
    m_ledMode = m_table.getEntry("ledMode");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_ta = m_table.getEntry("ta");
  }

  /** Register our NetworkTables server with Limelight proxy on the Raspberry Pi */
  private static void registerForNetworkTables() {
    String pi = System.getenv("HALSIMWS_HOST");

    try (Socket socket = new Socket(pi, LimelightNTRegistrationPort)) {
        BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        String response = reader.readLine();
        if (response.length() == 1 && response.getBytes()[0] == '0') {
          System.out.println("Registered with Romi Limelight proxy");
          return; // success
        }
        System.err.println(response);
        while ((response = reader.readLine()) != null)
            System.err.println(response);
        throw new ProtocolException("Failed to setup forwarding on the Pi for Limelight NetworkTables traffic");           
    } catch (UnknownHostException ex) {
        System.err.println("Server not found in RomiLimelight::registerForNetworkTables: " + ex.getMessage()); 
    } catch (IOException ex) {
        System.err.println("I/O error in RomiLimelight::registerForNetworkTables: " + ex.getMessage());
    }
  }
}
