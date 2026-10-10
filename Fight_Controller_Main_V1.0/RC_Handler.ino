void update_channels()
{
  crsf.update();
  for (int i = 1; i <= NUM_CHANNELS; i++)
  {
    pwm[i] = crsf.getChannel(i);
    duty[i] = map(pwm[i], 1000, 2000, -100, 100); //channels can be accessed by calling duty[0] for channel 1 etc
  }
  // Announce ourselves to the CRSF router once per second
  if (millis() - lastHeartbeatMs > 1000)
  {
    lastHeartbeatMs = millis();
    crsf.sendHeartbeat();
  }

  if (millis() - lastPrintMs > 1000)
  {
    lastPrintMs = millis();
    printEverything();  //for debugging
  }
}

void printEverything()
{
  Serial.println("----------------------------------------");
  Serial.print("link: ");
  Serial.print(crsf.isLinkUp() ? "UP" : "DOWN");
  const crsfLinkStatistics_t *link = crsf.getLinkStatistics();
  Serial.print("  LQ: ");
  Serial.print(link->uplink_Link_quality);
  Serial.print("  RSSI: -");
  Serial.print(link->active_antenna == 0 ? link->uplink_RSSI_1 : link->uplink_RSSI_2);
  Serial.print("dBm  armed: ");
  Serial.println(crsf.isArmed() ? "yes" : "no");

  Serial.print("channels 1-8:");
  for (int i = 1; i <= 8; i++)
  {
    Serial.print(" ");
    Serial.print(crsf.getChannel(i));
  }
  Serial.println("");

  // ELRS 4.0: VBAT receivers send millivolt-precision voltage as CELLS
  const crsf_sensor_cells_t *cells = crsf.getCellsSensor();
  if (cells->cell_count > 0)
  {
    Serial.print("cells (source ");
    Serial.print(cells->source_id);
    Serial.print("):");
    for (int i = 0; i < cells->cell_count; i++)
    {
      Serial.print(" ");
      Serial.print(cells->cell[i]);
      Serial.print("mV");
    }
    Serial.println("");
  }

  const crsf_sensor_temp_t *temp = crsf.getTempSensor();
  if (temp->temp_count > 0)
  {
    Serial.print("temps (deci-C):");
    for (int i = 0; i < temp->temp_count; i++)
    {
      Serial.print(" ");
      Serial.print(temp->temperature[i]);
    }
    Serial.println("");
  }

  const crsf_sensor_rpm_t *rpm = crsf.getRpmSensor();
  if (rpm->rpm_count > 0)
  {
    Serial.print("rpm:");
    for (int i = 0; i < rpm->rpm_count; i++)
    {
      Serial.print(" ");
      Serial.print(rpm->rpm[i]);
    }
    Serial.println("");
  }

  if (crsf.getAirspeedSensor()->speed != 0)
  {
    Serial.print("airspeed: ");
    Serial.print(crsf.getAirspeedSensor()->speed / 10.0);
    Serial.println(" km/h");
  }

  const crsf_sensor_gps_time_t *gpsTime = crsf.getGpsTimeSensor();
  if (gpsTime->year != 0)
  {
    Serial.print("gps time: ");
    Serial.print(gpsTime->year);
    Serial.print("-");
    Serial.print(gpsTime->month);
    Serial.print("-");
    Serial.print(gpsTime->day);
    Serial.print(" ");
    Serial.print(gpsTime->hour);
    Serial.print(":");
    Serial.print(gpsTime->minute);
    Serial.print(":");
    Serial.println(gpsTime->second);
  }
}