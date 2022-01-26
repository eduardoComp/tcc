
#define PWM_MC 5
#define PWM_MB 6

#define IN1_MC 8
#define IN2_MC 9

#define IN1_MB 12
#define IN2_MB 13

#define INTERRUPT_MB 1 // Pin 3
#define INTERRUPT_MC 0 // Pin 2

#define SWITCH_ROLL 4 // 0 - 800 pulses
#define SWITCH_PITCH 7

#define mechanical_limit_pitch 12

// Counter of pulses
volatile long int encoder_mc = 0;
volatile char acc_mc = '+';

volatile long int encoder_mb = 0;
volatile char acc_mb = '+';

// Count how many times the position change
int count_change_position = 0;

// receve the position from serial
int pos = 0;

// flag that sinalize the achive the target position
boolean end_mb = false;
boolean end_mc = false;

void count_mc()
{
  if (acc_mc == '+')
    encoder_mc++;
  else if (acc_mc == '-')
    encoder_mc--;
  else
    return;
}

void count_mb()
{
  if (acc_mb == '+')
    encoder_mb++;
  else if (acc_mb == '-')
    encoder_mb--;
  else
    return;
}

void setup()
{
  Serial.begin(9600);

  pinMode(PWM_MC, OUTPUT);
  pinMode(IN1_MC, OUTPUT);
  pinMode(IN2_MC, OUTPUT);


  pinMode(PWM_MB, OUTPUT);
  pinMode(IN1_MB, OUTPUT);
  pinMode(IN2_MB, OUTPUT);

  pinMode(mechanical_limit_pitch, INPUT);
  
  pinMode(SWITCH_PITCH, INPUT);
  digitalWrite(SWITCH_PITCH, HIGH); // turn on the resistor pull up fro remove jitters

  pinMode(SWITCH_ROLL, INPUT);
  digitalWrite(SWITCH_ROLL, HIGH); // turn on the resistor pull up fro remove jitters
  
  pinMode(INTERRUPT_MC, INPUT);
  pinMode(INTERRUPT_MB, INPUT);

  
  attachInterrupt(INTERRUPT_MC, count_mc, RISING);
  attachInterrupt(INTERRUPT_MB, count_mb, RISING);
}


void motorPos(int posT, String d) {

//  if (digitalRead(mechanical_limit_pitch)) {
//    return;
//  }
  
  acc_mb = '+';
  acc_mc = '+';

  count_change_position = 0;
  
  encoder_mb = 0;
  encoder_mc = 0;

  // Velocity
  int v = 255;

  // Motor B
  float post_mb = posT;
  float high2_mb = posT + posT * 0.3;
  float low2_mb = posT - posT * 0.3;
  float high_mb = posT + posT * 0.05;
  float low_mb = posT - posT * 0.05;

  // Motor C
  float post_mc = posT * 2;
  float high2_mc = post_mc + post_mc * 0.3;
  float low2_mc = post_mc - post_mc * 0.3;
  float high_mc = post_mc + post_mc * 0.05;
  float low_mc = post_mc - post_mc * 0.05;

  if (d.equals("up")) {
    digitalWrite(IN1_MB, HIGH);
    digitalWrite(IN2_MB, LOW);
    digitalWrite(IN1_MC, HIGH);
    digitalWrite(IN2_MC, LOW);
  }
  else if (d.equals("down")) {
    digitalWrite(IN1_MB, LOW);
    digitalWrite(IN2_MB, HIGH);
    digitalWrite(IN1_MC, LOW);
    digitalWrite(IN2_MC, HIGH);
  }
  else if (d.equals("+roll")) {
    digitalWrite(IN1_MB, HIGH);
    digitalWrite(IN2_MB, LOW);
    digitalWrite(IN1_MC, LOW);
    digitalWrite(IN2_MC, HIGH);
  }  
  else if (d.equals("-roll")) {
    digitalWrite(IN1_MB, LOW);
    digitalWrite(IN2_MB, HIGH);
    digitalWrite(IN1_MC, HIGH);
    digitalWrite(IN2_MC, LOW);
  }
  else {
    return;
  }

  // Aux to achive the position
  boolean lock_mb = false;
  boolean lock_mc = false;

  analogWrite(PWM_MC, v);
  analogWrite(PWM_MB, v);

  while (!end_mb || !end_mc) {


    
    if (low_mc <= encoder_mc && encoder_mc <= high_mc) {
      digitalWrite(IN1_MC, HIGH);
      digitalWrite(IN2_MC, HIGH);
      end_mc = true;
    }
    if (low_mb <= encoder_mb && encoder_mb <= high_mb) {
      digitalWrite(IN1_MB, HIGH);
      digitalWrite(IN2_MB, HIGH);
      end_mb = true;
    }

    if (!end_mb) { lock_mb = false; }
    if (!end_mc) { lock_mc = false; }
    

    while (!lock_mb || !lock_mc) {
      
      // Achive the mechanical limits
      if (d.equals("down") && !digitalRead(SWITCH_PITCH)) {
          digitalWrite(IN1_MB, HIGH);
          digitalWrite(IN2_MB, HIGH);
          digitalWrite(IN1_MC, HIGH);
          digitalWrite(IN2_MC, HIGH);
          end_mb = false;
          end_mc = false;
          Serial.print("ok");
          return;
      }
      // Entered loop
      else if (count_change_position > 4) {
          digitalWrite(IN1_MB, HIGH);
          digitalWrite(IN2_MB, HIGH);
          digitalWrite(IN1_MC, HIGH);
          digitalWrite(IN2_MC, HIGH);
          end_mb = false;
          end_mc = false;
          Serial.print("ok");
          return;
      }
      
      Serial.print("foward - B : ");
      Serial.println(encoder_mb);
      Serial.print("foward - C : ");
      Serial.println(encoder_mc);
    
      if (post_mb < encoder_mb) {
        digitalWrite(IN1_MB, HIGH);
        digitalWrite(IN2_MB, HIGH);
        lock_mb = true;
      }
      
      if (post_mc < encoder_mc) {
        digitalWrite(IN1_MC, HIGH);
        digitalWrite(IN2_MC, HIGH);
        lock_mc = true;
      }

      if (acc_mb == '-' || acc_mc == '-'){
        count_change_position++;
      }
      
      // UP
      if (acc_mb != '+' && !lock_mb) {
        digitalWrite(IN1_MB, HIGH);
        digitalWrite(IN2_MB, LOW);
        acc_mb = '+';
      }
      if (acc_mc != '+' && !lock_mc) {
        digitalWrite(IN1_MC, HIGH);
        digitalWrite(IN2_MC, LOW);
        acc_mc = '+';
      }
      delay(10);
    }

    if (!end_mb) { lock_mb = false; }
    if (!end_mc) { lock_mc = false; }

    while (!lock_mb || !lock_mc) {

      // Achive the mechanical limits
      if (d.equals("down") && !digitalRead(SWITCH_PITCH)) {
          //Serial.println("Limite mecanico atingido");
          digitalWrite(IN1_MB, HIGH);
          digitalWrite(IN2_MB, HIGH);
          digitalWrite(IN1_MC, HIGH);
          digitalWrite(IN2_MC, HIGH);
          end_mb = false;
          end_mc = false;
          Serial.print("ok");
          return;
      }
      // Entered loop
      else if (count_change_position > 4) {
          //Serial.println("Entrou em loop");
          digitalWrite(IN1_MB, HIGH);
          digitalWrite(IN2_MB, HIGH);
          digitalWrite(IN1_MC, HIGH);
          digitalWrite(IN2_MC, HIGH);
          end_mb = false;
          end_mc = false;
          Serial.print("ok");
          return;
      }
      if (post_mb >= encoder_mb) {
        digitalWrite(IN1_MB, HIGH);
        digitalWrite(IN2_MB, HIGH);
        lock_mb = true;
      }
      
      if (post_mc >= encoder_mc) {
        digitalWrite(IN1_MC, HIGH);
        digitalWrite(IN2_MC, HIGH);
        lock_mc = true;
      }

      if (acc_mb == '+' || acc_mc == '+'){
        count_change_position++;
      }
      
      // Down
      if (acc_mb != '-' && !lock_mb) {
        digitalWrite(IN1_MB, LOW);
        digitalWrite(IN2_MB, HIGH);
        acc_mb = '-';
      }
      if (acc_mc != '-' && !lock_mc) {
        digitalWrite(IN1_MC, LOW);
        digitalWrite(IN2_MC, HIGH);
        acc_mc = '-';
      }
      delay(10);
    }
    delay(10);
  }

  end_mb = false;
  end_mc = false;

  Serial.print("ok");
}

void homeRoll(String dir) {

   int v = 255;

   
  analogWrite(PWM_MC, v);
  analogWrite(PWM_MB, v);
  
   if (dir.equals("+roll")) {
    digitalWrite(IN1_MB, HIGH);
    digitalWrite(IN2_MB, LOW);
    digitalWrite(IN1_MC, LOW);
    digitalWrite(IN2_MC, HIGH);
  }  
  else if (dir.equals("-roll")) {
    digitalWrite(IN1_MB, LOW);
    digitalWrite(IN2_MB, HIGH);
    digitalWrite(IN1_MC, HIGH);
    digitalWrite(IN2_MC, LOW);
  } else {
    return;
  }

  while(1) {
    if (!digitalRead(SWITCH_ROLL)) {
      digitalWrite(IN1_MB, HIGH);
      digitalWrite(IN2_MB, HIGH);
      digitalWrite(IN1_MC, HIGH);
      digitalWrite(IN2_MC, HIGH);
      return;
    }
  }
}


void loop()
{
  
  if(Serial.available()>0){
    String text = Serial.readString();

    int str_len = text.length()+1;
    char cmd[str_len];
    
    text.toCharArray(cmd, str_len);
    char* p = cmd;

    int posT = String(strtok(p, ";")).toInt();
    p = NULL;
    String dir = String(strtok(p, ";"));

   if (posT>0 && (dir.equals("up") || dir.equals("down") || dir.equals("+roll") || dir.equals("-roll"))) {
        posT = map(posT, 0, 360, 0, 400);
        motorPos(posT, dir);
    } 
  }
}
