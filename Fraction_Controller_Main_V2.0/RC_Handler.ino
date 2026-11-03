void update_channels()
{
  crsf.update();
  if(crsf.isLinkUp()) //checks if TX is connected
  {
    rc_status = true;
  }
  else
  {
    rc_status = false;
  }
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
    printEverythingESC();
  }
}

// CRSF frame types used below (from the CRSF spec)
const uint8_t FT_BATTERY = 0x08; // battery sensor: voltage, current, capacity, remaining %
const uint8_t FT_RPM     = 0x0C; // rpm sensor

// The radio found no RPM or temperature sensors, so by default we send
// everything inside the battery frame (see sendBattery below). Set this to 1 to
// ALSO send the standard temperature and RPM frames, e.g. after updating EdgeTX
// to see if the radio discovers them. It costs a little telemetry bandwidth.
#define SEND_SEPARATE_TEMP_RPM 0

void send_telem()
{
  if (millis() - lastTelemMs >= TELEM_INTERVAL_MS)
  {
    lastTelemMs = millis();

    // Hottest of the three ESCs, 0 until temperature data arrives
    float hottest = 0;
    for (int i = 0; i < 3; i++)
    {
      if (!isnan(esc_temp[i]) && esc_temp[i] > hottest) hottest = esc_temp[i];
    }

    // Everything you asked for, in the one frame the radio shows reliably
    sendBattery(pack_volts, pack_amps, esc_rpm[IDX_W], hottest);

#if SEND_SEPARATE_TEMP_RPM
    // ESC temperatures in tenths of a degree C: right, left, weapon
    int16_t temperatures[3] = { toTenths(esc_temp[IDX_R]), toTenths(esc_temp[IDX_L]), toTenths(esc_temp[IDX_W]) };
    sendTemperature(0, temperatures, 3);

    // Motor RPM: right, left, weapon
    sendRpm(0, esc_rpm, 3);
#endif
  }
}

// Float to tenths as an integer. NAN (no data yet) becomes 0.
int16_t toTenths(float v)
{
  if (isnan(v)) return 0;
  return (int16_t)roundf(v * 10.0f);
}

// Battery frame, with the two spare fields borrowed for RPM and temperature.
// The radio shows four sensors from it. Rename them on the radio's Telemetry page:
//   RxBt = battery volts           (real, 0.1 V steps)
//   Curr = HOTTEST ESC temperature (the radio labels it A, value is degrees C, 0.1 steps)
//   Capa = WEAPON RPM              (the radio labels it mAh, ignore that; value is RPM)
//   Bat% = total current in AMPS   (the radio labels it %, value is amps, whole amps)
// A value of 0 means "no data yet" (ESC not replying or EDT not started).
// The radio reads the Bat% byte as signed, so current is capped at 127 A there.
// Temperature has its own 16 bit field, so it can read well past 140 C.
void sendBattery(float volts, float amps, float weaponRpm, float hottestC)
{
  uint16_t dv = (uint16_t)constrain(toTenths(volts), 0, 32767);
  uint16_t dt = (uint16_t)constrain(toTenths(hottestC), 0, 32767); // 0.1 C steps

  // RPM goes in the 3-byte capacity field (max 8,388,607, far more than needed)
  long rpm = isnan(weaponRpm) ? 0 : lroundf(weaponRpm);
  rpm = constrain(rpm, 0L, 8388607L);

  long amp = isnan(amps) ? 0 : lroundf(amps);
  amp = constrain(amp, 0L, 127L);

  uint8_t payload[8];
  payload[0] = (dv >> 8) & 0xFF;   // voltage, MSB first
  payload[1] = dv & 0xFF;
  payload[2] = (dt >> 8) & 0xFF;   // "current" field = hottest ESC temp, MSB first
  payload[3] = dt & 0xFF;
  payload[4] = (rpm >> 16) & 0xFF; // "capacity" = weapon RPM, 24 bit, MSB first
  payload[5] = (rpm >> 8) & 0xFF;
  payload[6] = rpm & 0xFF;
  payload[7] = (uint8_t)amp;       // "remaining %" = total current in whole amps

  crsf.queuePacket(CRSF_SYNC_BYTE, FT_BATTERY, payload, 8);
}

// RPM frame: a source id, then each RPM as a 24 bit number, MSB first.
void sendRpm(uint8_t sourceId, const float *values, uint8_t count)
{
  if (count > 8)
    count = 8;

  uint8_t payload[1 + 8 * 3];
  payload[0] = sourceId;

  for (uint8_t i = 0; i < count; i++)
  {
    int32_t r = isnan(values[i]) ? 0 : (int32_t)roundf(values[i]);
    payload[1 + i*3]     = (r >> 16) & 0xFF;
    payload[1 + i*3 + 1] = (r >> 8) & 0xFF;
    payload[1 + i*3 + 2] = r & 0xFF;
  }

  crsf.queuePacket(CRSF_SYNC_BYTE, FT_RPM, payload, 1 + count*3);
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

void sendTemperature(uint8_t sourceId, const int16_t *values, uint8_t count)
{
  if (count > CRSF_MAX_TEMP_VALUES)
    count = CRSF_MAX_TEMP_VALUES;

  uint8_t payload[1 + CRSF_MAX_TEMP_VALUES * 2];
  payload[0] = sourceId;

  for (uint8_t i = 0; i < count; i++)
  {
    // Values are MSB first (BigEndian)
    payload[1 + i*2]     = (values[i] >> 8) & 0xFF;
    payload[1 + i*2 + 1] = values[i] & 0xFF;
  }

  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_TEMP, payload, 1 + count*2);
}
