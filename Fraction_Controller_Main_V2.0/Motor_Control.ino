void update_motors()
{

  if ((esp_timer_get_time() - motor_update) < 500) return;
  motor_update = esp_timer_get_time();
	int thr  = duty[2];  // whichever channels you use
	int steer = duty[1];
	int wep = duty[3];
	if (duty[5] > 80)
	{
		wep_reversed = true;
	}
	else
	{
		wep_reversed = false;
	}
	motor_L = constrain(-thr - steer, -100, 100);
	motor_R = constrain(-thr + steer, -100, 100);
	motor_W = map(wep, -100, 100, 0, 100);
	if (wep_reversed == true)
	{
		motor_W = motor_W * -1;
	}

	// FAILSAFE: with no radio link duty[] is all 0, and map(0, -100, 100, 0, 100) = 50,
	// so without this the weapon would spin at 50% whenever the link is down.
	if (rc_status == false)
	{
		motor_L = 0;
		motor_R = 0;
		motor_W = 0;
	}

	// Convert to DShot values first, so we can check for "stopped" before sending
	int dshot_R = throttle3D(motor_R);
	int dshot_L = throttle3D(motor_L);
	int dshot_W = throttle3D(motor_W);

	// Turn on extended telemetry (temp / voltage / current). Only does anything
	// while that ESC is stopped, see enable_edt() below.
	enable_edt(esc_R, dshot_R, IDX_R);
	enable_edt(esc_L, dshot_L, IDX_L);
	enable_edt(esc_W, dshot_W, IDX_W);

	esc_R.send(dshot_R);
	esc_L.send(dshot_L);
	esc_W.send(dshot_W);
}

// Asks one ESC to start sending extended DShot telemetry (EDT).
// - "dshot_value" is what you are about to send to that ESC. 0 means stopped.
// - "i" is IDX_R, IDX_L or IDX_W (just used to remember when we last asked).
// The command briefly takes the place of the throttle, so it is only sent while
// the motor is stopped. It is repeated once a second until EDT data shows up,
// because the ESC ignores it if it is still booting. Once EDT is working this
// function does nothing.
void enable_edt(AlfredoDShot &esc, int dshot_value, int i)
{
  if (dshot_value != 0) return;              // motor running, don't interrupt it
  if (!esc.telemetryValid()) return;         // ESC isn't replying yet
  if (esc.commandPending()) return;          // previous command still going out
  if (esc.edtSeen()) return;                 // EDT already working
  if (millis() - edt_sent[i] < 1000) return; // asked less than 1 s ago, wait

  esc.command(DSHOT_CMD_EDT_ENABLE);
  edt_sent[i] = millis();
}

// Copies the latest telemetry from one ESC into the esc_* arrays.
// temp / volts / amps are NAN until the first EDT data arrives.
void read_esc(AlfredoDShot &esc, int i)
{
  esc_rpm[i]   = esc.telemetryValid() ? esc.rpm() : NAN;
  esc_temp[i]  = esc.temperatureC();
  esc_volts[i] = esc.voltage();
  esc_amps[i]  = esc.current() * CURRENT_SCALE;   // see CURRENT_SCALE in the main tab
}

void get_telem()
{
  read_esc(esc_R, IDX_R);
  read_esc(esc_L, IDX_L);
  read_esc(esc_W, IDX_W);

  // All three ESCs sit on the same battery, so use the first voltage we have
  pack_volts = NAN;
  for (int i = 0; i < 3; i++)
  {
    if (!isnan(esc_volts[i]))
    {
      pack_volts = esc_volts[i];
      break;
    }
  }

  // Total current = sum of whichever ESCs are reporting
  pack_amps = 0;
  for (int i = 0; i < 3; i++)
  {
    if (!isnan(esc_amps[i])) pack_amps += esc_amps[i];
  }
}

void printEscTelem(const char *name, int i)
{
  Serial.printf("%s  rpm %.0f  temp %.0fC  %.1fV  %.1fA\n",
                name, esc_rpm[i], esc_temp[i], esc_volts[i], esc_amps[i]);
}

void printEverythingESC()
{
	Serial.print("Right ESC Throttle:");
	Serial.print(motor_R);
	Serial.print("\tLeft ESC Throttle:");
	Serial.print(motor_L);
	Serial.print("\tWep ESC Throttle:");
	Serial.print(motor_W);
	Serial.print("\tLast DShot value:");
	Serial.println(throttle_print);
	printEscTelem("R", IDX_R);
	printEscTelem("L", IDX_L);
	printEscTelem("W", IDX_W);
}

int throttle3D(int16_t throttle)
{
  throttle = throttle * 10;
	//clamp throttle between -99 and 99
	if (throttle < 80 && throttle > -80)
	{
		throttle = 0;
	}
	if (throttle > 999)
	{
		throttle = 999;
	}
	if (throttle < -999)
	{
		throttle = -999;
	}
	//if throttle is negative, wrap it from -999 to 0 to 1000 to 1999
	if (throttle < 0)
	{
		throttle = 1000-throttle;
	}
  if (throttle > 2047 - 48)
	{
		throttle = 2047; //clamps to max throttle
	}
	else
	{
		throttle += 48; //offsets by dshot min
	}
	if (throttle == 48)
	{
		throttle = 0;
	}
	throttle_print = throttle;
	return throttle;
}
