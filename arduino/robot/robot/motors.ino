#define HISTORY_LENGTH 4

//-----------------------------------------------------------------------------
double filter(double pitch) {

  uint8_t pwmMapped;
  static bool filterActive=FALSE;
  static uint8_t posInArray, i;
  static int8_t pitchHistory[HISTORY_LENGTH];
  double pitchFiltered, prevPitch;

  if (!filterActive) {
    if (abs(pitch) < 1) {
      // Turn on filter and initialize history.
      filterActive = TRUE;
      for (i=0; i<HISTORY_LENGTH; ++i) {
        pitchHistory[i] = prevPitch;
      }
      pitchHistory[0] = pitch;
      posInArray = 0;
    } else {
      prevPitch = pitch;
      return pitch;
    }
      
  } else {
    // Filter is currently active.
    ++posInArray;
    if (posInArray >= HISTORY_LENGTH)
      posInArray = 0;
    pitchHistory[posInArray] = pitch;
  }

  pitchFiltered = 0;
  for (i=0; i<HISTORY_LENGTH; ++i) {
    pitchFiltered += pitchHistory[i];
  }
  pitchFiltered /= HISTORY_LENGTH;

  if (abs(pitchFiltered) > 1.0) {
    // Pitch has moved out of region where filter is active.
    filterActive = FALSE;
  }

  prevPitch = pitch;
  return pitchFiltered;
}

#if 0
//-----------------------------------------------------------------------------
void setMotors(int16_t pwm) {

  uint8_t pwmMapped;
  static bool prevDir;

  if (pwm >= 0) {  // Forward
    if (prevDir == FORWARD) {
      printt && Serial.print("  f");
      analogWrite(IN2_L, 0);                        
      analogWrite(IN2_R, 0);                        
      pwmMapped = map(pwm,0,255,10,255);
      analogWrite(IN1_L, pwmMapped);                        
      analogWrite(IN1_R, pwmMapped);                        
    } else {
      printt && Serial.print("  s");
      analogWrite(IN2_L, 0);                        
      analogWrite(IN2_R, 0);                        
      analogWrite(IN1_L, 0);                        
      analogWrite(IN1_R, 0);                        
      prevDir = FORWARD;
    }

  } else {  // Reverse
    if (prevDir == REVERSE) {
      printt && Serial.print("  r");
      analogWrite(IN1_L, 0);                        
      analogWrite(IN1_R, 0);                        
      pwm = abs(pwm);
      pwmMapped = map(pwm,0,255,10,255);
      analogWrite(IN2_L, pwmMapped);                        
      analogWrite(IN2_R, pwmMapped);                        
    } else {
      printt && Serial.print("  s");
      analogWrite(IN2_L, 0);                        
      analogWrite(IN2_R, 0);                        
      analogWrite(IN1_L, 0);                        
      analogWrite(IN1_R, 0);                        
      prevDir = REVERSE;
    }
  }

  printt && Serial.print("  pwmM=");
  printt && Serial.print(pwmMapped);
}

#endif

//-----------------------------------------------------------------------------
void updateWheelCountA_L() {
  int32_t enca, encb;

  enca = digitalRead(ENCA_L);
  encb = digitalRead(ENCB_L);

  if (enca == HIGH) {
    if (encb == LOW) {
      ++pulses_L;
    } else {
      --pulses_L;
    }
  } else {
    if (encb == HIGH) {
      ++pulses_L;
    } else {
      --pulses_L;
    }
  }

  //Serial.print("  p_L=");
  //Serial.print(pulses_L);
}


//-----------------------------------------------------------------------------
void updateWheelCountB_L() {
  int32_t enca, encb;

  enca = digitalRead(ENCA_L);
  encb = digitalRead(ENCB_L);

  if (encb == HIGH) {
    if (enca == HIGH) {
      ++pulses_L;
    } else {
      --pulses_L;
    }
  } else {
    if (enca == LOW) {
      ++pulses_L;
    } else {
      --pulses_L;
    }
  }

  //Serial.print("  p_L=");
  //Serial.print(pulses_L);
}


//-----------------------------------------------------------------------------
void updateWheelCountA_R() {
  int32_t enca, encb;

  enca = digitalRead(ENCA_R);
  encb = digitalRead(ENCB_R);

  if (enca == HIGH) {
    if (encb == HIGH) {
      ++pulses_R;
    } else {
      --pulses_R;
    }
  } else {
    if (encb == LOW) {
      ++pulses_R;
    } else {
      --pulses_R;
    }
  }

  //Serial.print("  p_R=");
  //Serial.print(pulses_R);
}


//-----------------------------------------------------------------------------
void updateWheelCountB_R() {
  int32_t enca, encb;

  enca = digitalRead(ENCA_R);
  encb = digitalRead(ENCB_R);

  if (encb == HIGH) {
    if (enca == LOW) {
      ++pulses_R;
    } else {
      --pulses_R;
    }
  } else {
    if (enca == HIGH) {
      ++pulses_R;
    } else {
      --pulses_R;
    }
  }

  //Serial.print("  p_R=");
  //Serial.print(pulses_R);
}


//-----------------------------------------------------------------------------
void setMotor_L(int16_t pwm) {

  uint8_t pwmMapped;

  if (pwm >= 0) {  // Forward
    printt && Serial.print("  f");
    analogWrite(IN2_L, 0);                        
    pwmMapped = map(pwm,0,255,3,255);
    analogWrite(IN1_L, pwmMapped);                        

  } else {  // Reverse
    printt && Serial.print("  r");
    analogWrite(IN1_L, 0);                        
    pwm = abs(pwm);
    pwmMapped = map(pwm,0,255,0,255);
    analogWrite(IN2_L, pwmMapped);                        
  }

  //printt && Serial.print("  pwm_L=");
  //printt && Serial.print(pwmMapped);
}


//-----------------------------------------------------------------------------
void setMotor_R(int16_t pwm) {

  uint8_t pwmMapped;

  if (pwm >= 0) {  // Forward
    analogWrite(IN2_R, 0);                        
    pwmMapped = map(pwm,0,255,4,255);
    analogWrite(IN1_R, pwmMapped);                        

  } else {  // Reverse
    analogWrite(IN1_R, 0);                        
    pwm = -pwm;
    pwmMapped = map(pwm,0,255,4,255);
    analogWrite(IN2_R, pwmMapped);                        
  }

  //printt && Serial.print("  pwm_R=");
  //printt && Serial.print(pwmMapped);
}



#if 0
//-----------------------------------------------------------------------------
int Drive_Motor(int pwm)  {

  uint8_t pwm_l, pwm_r;

  if (pwm >= 0) {  // drive motors forward
    printt && Serial.print("  f");
    digitalWrite(IN1_R, HIGH);                        
    digitalWrite(IN2_R, LOW);
    digitalWrite(IN1_L, HIGH);                     
    digitalWrite(IN2_L, LOW);

  } else {  // drive motors backward
    printt && Serial.print("  r");
    digitalWrite(IN1_R, LOW);                       
    digitalWrite(IN2_R, HIGH);
    digitalWrite(IN1_L, LOW);                      
    digitalWrite(IN2_L, HIGH);
    pwm = -pwm;
  }

  pwm_l = map(pwm,0,255,60,255);
  pwm_r = map(pwm,0,255,65,255);
  //if(pwm>5) map(pwm,0,255,30,255);

  analogWrite(D2n_L,pwm_l);
  analogWrite(D2n_R,pwm_r);
  
  printt && Serial.print("  pwm_l=");
  printt && Serial.print(pwm_l);
  printt && Serial.print("  pwm_r=");
  printt && Serial.print(pwm_r);
}
#endif
