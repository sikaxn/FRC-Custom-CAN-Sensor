package first.robot.subsystems;

import first.robot.drivers.ScWitMotionDriver;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;

public class WitMotionSubsystem extends SubsystemBase {
  private final NetworkTableEntry connectedEntry;
  private final NetworkTableEntry freshEntry;
  private final NetworkTableEntry staleEntry;
  private final NetworkTableEntry timestampMsEntry;
  private final NetworkTableEntry ageMsEntry;
  private final NetworkTableEntry sequenceEntry;
  private final NetworkTableEntry sampleRateEntry;
  private final NetworkTableEntry rollEntry;
  private final NetworkTableEntry pitchEntry;
  private final NetworkTableEntry yawEntry;
  private final NetworkTableEntry accelXEntry;
  private final NetworkTableEntry accelYEntry;
  private final NetworkTableEntry accelZEntry;
  private final NetworkTableEntry gyroXEntry;
  private final NetworkTableEntry gyroYEntry;
  private final NetworkTableEntry gyroZEntry;
  private final NetworkTableEntry temperatureEntry;
  private final NetworkTableEntry deviceEntry;
  private final NetworkTableEntry baudEntry;
  private final NetworkTableEntry serviceErrorEntry;
  private final NetworkTableEntry driverErrorEntry;
  private final ScWitMotionDriver witMotionDriver = new ScWitMotionDriver();

  public WitMotionSubsystem() {
    NetworkTable witMotionTable =
        NetworkTableInstance.getDefault().getTable("IMDemo").getSubTable("WitMotion");

    connectedEntry = witMotionTable.getEntry("Connected");
    freshEntry = witMotionTable.getEntry("Fresh");
    staleEntry = witMotionTable.getEntry("Stale");
    timestampMsEntry = witMotionTable.getEntry("TimestampMs");
    ageMsEntry = witMotionTable.getEntry("AgeMs");
    sequenceEntry = witMotionTable.getEntry("Sequence");
    sampleRateEntry = witMotionTable.getEntry("SampleRateHz");
    rollEntry = witMotionTable.getEntry("RollDegrees");
    pitchEntry = witMotionTable.getEntry("PitchDegrees");
    yawEntry = witMotionTable.getEntry("YawDegrees");
    accelXEntry = witMotionTable.getEntry("AccelXG");
    accelYEntry = witMotionTable.getEntry("AccelYG");
    accelZEntry = witMotionTable.getEntry("AccelZG");
    gyroXEntry = witMotionTable.getEntry("GyroXDps");
    gyroYEntry = witMotionTable.getEntry("GyroYDps");
    gyroZEntry = witMotionTable.getEntry("GyroZDps");
    temperatureEntry = witMotionTable.getEntry("TemperatureC");
    deviceEntry = witMotionTable.getEntry("Device");
    baudEntry = witMotionTable.getEntry("Baud");
    serviceErrorEntry = witMotionTable.getEntry("ServiceError");
    driverErrorEntry = witMotionTable.getEntry("DriverPollError");

    connectedEntry.setBoolean(false);
    freshEntry.setBoolean(false);
    staleEntry.setBoolean(true);
    timestampMsEntry.setInteger(0L);
    ageMsEntry.setInteger(0L);
    sequenceEntry.setInteger(0L);
    sampleRateEntry.setDouble(0.0);
    rollEntry.setDouble(Double.NaN);
    pitchEntry.setDouble(Double.NaN);
    yawEntry.setDouble(Double.NaN);
    accelXEntry.setDouble(Double.NaN);
    accelYEntry.setDouble(Double.NaN);
    accelZEntry.setDouble(Double.NaN);
    gyroXEntry.setDouble(Double.NaN);
    gyroYEntry.setDouble(Double.NaN);
    gyroZEntry.setDouble(Double.NaN);
    temperatureEntry.setDouble(Double.NaN);
    deviceEntry.setString("");
    baudEntry.setInteger(0L);
    serviceErrorEntry.setString("");
    driverErrorEntry.setString("");

    witMotionDriver.start();
  }

  public ScWitMotionDriver.Sample getLatestSample() {
    return witMotionDriver.getLatestSample();
  }

  @Override
  public void periodic() {
    ScWitMotionDriver.Sample sample = witMotionDriver.getLatestSample();

    connectedEntry.setBoolean(sample.isConnected());
    freshEntry.setBoolean(sample.isFresh());
    staleEntry.setBoolean(sample.isStale());
    timestampMsEntry.setInteger(sample.getTimestampMs());
    ageMsEntry.setInteger(sample.getAgeMs());
    sequenceEntry.setInteger(sample.getSequence());
    sampleRateEntry.setDouble(sample.getSampleRateHz());
    rollEntry.setDouble(sample.getRollDegrees());
    pitchEntry.setDouble(sample.getPitchDegrees());
    yawEntry.setDouble(sample.getYawDegrees());
    accelXEntry.setDouble(sample.getAccelXG());
    accelYEntry.setDouble(sample.getAccelYG());
    accelZEntry.setDouble(sample.getAccelZG());
    gyroXEntry.setDouble(sample.getGyroXDps());
    gyroYEntry.setDouble(sample.getGyroYDps());
    gyroZEntry.setDouble(sample.getGyroZDps());
    temperatureEntry.setDouble(sample.getTemperatureC());
    deviceEntry.setString(sample.getDevice());
    baudEntry.setInteger(sample.getBaud());
    serviceErrorEntry.setString(sample.getServiceError());
    driverErrorEntry.setString(witMotionDriver.getLastPollError());
  }
}
