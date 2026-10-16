void update_motors()
{

  if ((esp_timer_get_time() - motor_update) < 500) return;
  motor_update = esp_timer_get_time();
	int thr  = duty[2];  // whichever channels you use
	int steer = duty[1];
	motor_L = constrain(thr + steer, -100, 100);
	motor_R = constrain(thr - steer, -100, 100);
  esc_R.send(throttle3D(motor_R));  //send throttle -100 to 100
  esc_L.send(throttle3D(motor_L));
  esc_R_rpm = esc_R.rpm();
  esc_L_rpm = esc_L.rpm();
}

void printEverythingESC()
{
	Serial.print("Right ESC Throttle:");
	Serial.print(motor_R);
	Serial.print("\tLeft ESC Throttle:");
	Serial.print(motor_L);
	Serial.print("\tRight ESC RPM:");
	Serial.print(esc_R_rpm);
	Serial.print("\tLeft ESC RPM:");
	Serial.print(esc_L_rpm);
	Serial.print("\tThrottle");
	Serial.println(throttle_print);
}

int throttle3D(int16_t throttle)
{
  throttle = throttle * 10;
	//clamp throttle between -99 and 99
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
	throttle_print = throttle;
	return throttle;
}