////////////////////////////////////////////
//                 Main Loop              //
////////////////////////////////////////////


void loop() {
  if(x < 1000){
    //Delay the start of the motors to let the IMU calibrate
    getTheta(); 
    if (x % 100 == 0) {
      Serial.println((1000/100) - x/100);
    }
    x++;
  }else{
    // Read Motor Speeds
    getMotorSpeeds();
    // Calculate Theta and dTheta
    getTheta();
    // Let controller calculate set values for motors
    getPWM();
    // Set desired motor values
    setMotorSpeeds();
    // Print to Serial output for plotting
    printSerial();
  }
}


////////////////////////////////////////////
//         Motor Encoder Interrupts       //
////////////////////////////////////////////
void rpm_A_tach() {
  A_tach_rev_tick++;

  if (A_tach_rev_tick >= tach_rev_res) {
    A_time_prev = A_time_now;
    A_time_now = micros();
    A_time_diff = A_time_now - A_time_prev;

    A_tach_rev_tick = 0;
  }
}

void rpm_B_tach() {
  B_tach_rev_tick++;

  if (B_tach_rev_tick >= tach_rev_res) {
    B_time_prev = B_time_now;
    B_time_now = micros();
    B_time_diff = B_time_now - B_time_prev;

    B_tach_rev_tick = 0;
  }
}


////////////////////////////////////////////
//    Calculate Motor Speeds in rad/s     //
////////////////////////////////////////////
void getMotorSpeeds() {
  /// MOTOR A
  A_speed_prev = A_speed;
  
  //if time since last revolution is too big
  if ( (micros() - A_time_now) > (long)(500000.) ) {
    A_speed = 0;
  } else {
    // Calculate speed in rad/s
    A_speed = (1000000./A_time_diff) * 2*M_PI ;// * A_dir; 
    // Determine direction of speed
    if(A_dir != A_dir_set && abs(A_speed_prev) < 17 && abs(A_speed) > abs(A_speed_prev)){
      A_dir = A_dir_set;
    }
    // Set speed with direction
  //  A_speed = A_speed * A_dir;
    
  }
  A_speed = 0.3*A_speed + 0.7*A_speed_prev;

  /// MOTOR B
  B_speed_prev = B_speed;

  // Check if time since last revolution is too big
  if ( (micros() - B_time_now) > (long)(500000.) ) {
    B_speed = 0;
  } else {
    // Calculate speed in rad/s
    B_speed = 1000000./B_time_diff * 2*M_PI ;// * A_dir;
    // Determine direction of speed
    if(B_dir != B_dir_set && abs(B_speed_prev) < 17 && abs(B_speed) > abs(B_speed_prev)){
      B_dir = B_dir_set;
    }
    // Set speed with direction
    B_speed = B_speed * B_dir;
  }
  B_speed = 0.3*B_speed + 0.7*B_speed_prev;
}


////////////////////////////////////////////
//           Theta Calculations           //
////////////////////////////////////////////
void getTheta(){
  getIMUValues();
  // Calculations for theta_A and dTheta_A
  cycle_time = (float)(micros() - theta_time);
  theta_time = (float) micros();
  
  theta_A_prev = theta_A;
  dTheta_A_prev = dTheta_A;
  
  // Filter theta value
  theta_A = (float) (1-0.7)*angle_y + 0.7*theta_A_prev;
  
  // Find dTheta and apply filter
  dTheta_A = (float) (theta_A - theta_A_prev)/cycle_time * 1000000.;
  dTheta_A = (float) (1-0.7)*dTheta_A + 0.7*dTheta_A_prev;

  // Calculate theta error
  theta_A_error = theta_A_zero - theta_A;
  //theta_A_error = theta_A - theta_A_zero;
  
  /* ////// From Mike Rouleau's Self Balancing Stick /////////
   *  Moves the zero reference in order to compensate for center of mass
   *  not being at theta_B = 0
   *  Is also used if a weight is added to change center of mass
   */
  theta_A_zero_prev = theta_A_zero;
  theta_A_zero = theta_A_zero - 0.9 * theta_A_error;
  theta_A_zero = (1 - 0.995) * theta_A_zero + 0.995 * theta_A_zero_prev;
   /*
   * /////// End /////////////
   */
  
  // Calculations for theta_B and dTheta_B
  
  theta_B_prev = theta_B;
  dTheta_B_prev = dTheta_B;

  // Filter theta value and find error
  theta_B = (float) (1-0.7)*angle_x + 0.7*theta_B_prev;
  
  theta_B_error = theta_B_zero - theta_B;
  //theta_B_error = theta_B - theta_B_zero;
  
  // Calculate dTheta and apply filtering
  dTheta_B = (float) (theta_B - theta_B_prev)/cycle_time * 1000000.;
  dTheta_B = (float) (1-0.7)*dTheta_B + 0.7*dTheta_B_prev;


  /* ////// From Mike Rouleau's Self Balancing Stick /////////
   *  Moves the zero reference in order to compensate for center of mass
   *  not being at theta_B = 0
   *  Is also used if a weight is added to change center of mass
   */

  theta_B_zero_prev = theta_B_zero;
  theta_B_zero = theta_B_zero - 0.9 * theta_B_error;
  theta_B_zero = (1 - 0.995) * theta_B_zero + 0.995 * theta_B_zero_prev;

  /*
   * /////// End /////////////
   */
}


////////////////////////////////////////////
//        Controller Calculations         //
////////////////////////////////////////////
void getPWM(){
  // Channel A
  //Calculate control output
  A_set_speed = ((-Kp1_A*theta_A_error - Kp2_A*dTheta_A)-Kp3_A*A_speed);

  //Add speed to compensate for friction
//  if(A_set_speed > 20){
//    A_set_speed += 30;
//  } else if (A_set_speed < -20){
//    A_set_speed -= 30;
//  }
  
  // Constrain to map to PWM min/max
  // For the Pololu motors in use right now the setM1Speed command needs value between -400 and 400
  // The command then converts it to be between -255 and 255
 A_set_PWM = round(constrain(A_set_speed, -400, 400));

  // Set PWM = 0 for values that are too low
 // if(A_set_PWM < 20 && A_set_PWM > -20){
//    A_set_PWM = 0;
 // }else{
 //   A_set_PWM = (int) A_set_PWM;
 // }

  // Channel B
  //Calculate control output
  B_set_speed = ((-Kp1_B*theta_B_error - Kp2_B*dTheta_B)-Kp3_B*B_speed);

  //Add speed to compensate for friction
 // if(B_set_speed < 150){
//    B_set_speed += 30;
 // } else if (B_set_speed < -20){
 //   B_set_speed -= 30;
//  }
  
  //Constrain to map tp PWM min/max
  //  B_set_PWM = round(constrain(B_set_speed, -400, 400));

  //Set PWM = 0 for values that are too low
 // if(B_set_PWM < 20 && B_set_PWM > -20){
 //   B_set_PWM = 0;
//  }else{
 //   B_set_PWM = (int) B_set_PWM;
 // }
}
////////////////////////////////////////////
//    Set Motor output to desired value   //
////////////////////////////////////////////

void setMotorSpeeds(){
  // Set PWM for motor A
  
  if (A_set_PWM != A_set_PWM_prev){
    if (A_set_PWM > 0){
      digitalWrite(pin_A_dir, LOW); //Establishes direction of Channel A
      A_dir_set = 1;  // Direction motor is set
      digitalWrite(pin_A_brake, LOW);   //Disengage the Brake for Channel A
      analogWrite(pin_A_PMW, A_set_PWM);   //Spins the motor on Channel A
      md.setM1Speed(A_set_PWM);
    }else if (A_set_PWM < 0){
      digitalWrite(pin_A_dir, HIGH); //Establishes direction of Channel A
      A_dir_set = -1;
      digitalWrite(pin_A_brake, LOW);   //Disengage the Brake for Channel A
      analogWrite(pin_A_PMW, (abs(A_set_PWM)));   //Spins the motor on Channel A
      md.setM1Speed(A_set_PWM);
    }else{
      digitalWrite(pin_A_brake, HIGH); //Engage the Breke for Channel B
      analogWrite(pin_A_PMW, 0);
      md.setM2Speed(0);
    }
  }
  
  md.setM1Speed(A_set_PWM);
  // Set PWM for motor B
  
  if (B_set_PWM != B_set_PWM_prev){
    if (B_set_PWM > 0){
      digitalWrite(pin_B_dir, LOW); //Establishes direction of Channel B
      B_dir_set = 1;
      digitalWrite(pin_B_brake, LOW);   //Disengage the Brake for Channel B
      analogWrite(pin_B_PMW, B_set_PWM);   //Spins the motor on Channel B
      md.setM2Speed(B_set_PWM);
    }else if (B_set_PWM < 0){
      digitalWrite(pin_B_dir, HIGH); //Establishes direction of Channel B
      B_dir_set = -1;
      digitalWrite(pin_B_brake, LOW);   //Disengage the Brake for Channel B
      analogWrite(pin_B_PMW, (abs(B_set_PWM)));   //Spins the motor on Channel B
      md.setM2Speed(B_set_PWM);
    }else{
      digitalWrite(pin_B_brake, HIGH);  //Engage the Breke for Channel B
      analogWrite(pin_B_PMW, 0); 
      md.setM2Speed(0);
    }
  }
  
  md.setM2Speed(B_set_PWM);
  // Save last PWM values
  A_set_PWM_prev = A_set_PWM;
  B_set_PWM_prev = B_set_PWM;
}



////////////////////////////////////////////
//            Serial Print                //
////////////////////////////////////////////
void printSerial(){  
  package = micros(); 
  package += "\t";
  //package += A_speed;
  package += A_set_speed;
  package += "\t";
  //package += B_speed;
  package += B_set_speed;
  package += "\t";
  package += theta_A;
  package += "\t";
  package += theta_B;
  package += "\t";
  package += dTheta_A;
  package += "\t";
  package += dTheta_B;
  package += "\t";
  /*
  package += A_set_speed;
  package += "\t";
  package += B_set_speed;
  package += "\t";
  package += A_set_PWM;
  package += "\t";
  package += B_set_PWM;
  package += "\t";
  */
  package += theta_A_zero;
  package += "\t";
  package += theta_B_zero;
   package += "\t";
  package += angle_x; 
  package += "\t";
  package += angle_y;
  /*
  package += "\t";
  package += A_time_diff;
  package += "\t";
  package += B_time_diff;
  */
  Serial.println(package);
}
