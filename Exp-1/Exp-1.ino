#define RED 17
#define YEL 25
#define GRE 26


void setup() {
  pinMode(RED, OUTPUT);
  pinMode(YEL, OUTPUT);
  pinMode(GRE, OUTPUT);
}

void loop() {
  digitalWrite(RED, HIGH);
  digitalWrite(YEL, LOW);
  digitalWrite(GRE, LOW);
  delay(10000);

  digitalWrite(RED, LOW);
  for(int i=0; i<3; i++){
    digitalWrite(YEL, HIGH);
    delay(500);
    digitalWrite(YEL, LOW);
    delay(500);
  }

  digitalWrite(GRE, HIGH);
  delay(10000);
}
