// Pin assignments
#define RED_PIN    15
#define GREEN_PIN  13
#define BLUE_PIN   14

#define RELAY_PIN  25
#define BUTTON_RELAY  0   // IO0
#define BUTTON_LED    17  // IO17

bool relayState = false;
int ledState = 0;

void setup() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // relay off initially

  pinMode(BUTTON_RELAY, INPUT_PULLUP);
  pinMode(BUTTON_LED, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("Test program started");
}

void loop() {
  static bool prevRelayBtn = HIGH;
  static bool prevLedBtn = HIGH;

  bool relayBtn = digitalRead(BUTTON_RELAY);
  bool ledBtn = digitalRead(BUTTON_LED);

  // Toggle relay on falling edge (button press)
  if (prevRelayBtn == HIGH && relayBtn == LOW) {
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    Serial.print("Relay: ");
    Serial.println(relayState ? "ON" : "OFF");
  }

  // Toggle LED color on falling edge
  if (prevLedBtn == HIGH && ledBtn == LOW) {
    ledState = (ledState + 1) % 4;
    updateLED(ledState);
    Serial.print("LED State: ");
    Serial.println(ledState);
  }

  prevRelayBtn = relayBtn;
  prevLedBtn = ledBtn;

  delay(10); // debounce delay
}

void updateLED(int state) {
  switch (state) {
    case 0: setColor(0, 0, 0); break; // off
    case 1: setColor(255, 0, 0); break; // red
    case 2: setColor(0, 255, 0); break; // green
    case 3: setColor(0, 0, 255); break; // blue
  }
}

void setColor(int r, int g, int b) {
  digitalWrite(RED_PIN,   r > 0 ? HIGH : LOW);
  digitalWrite(GREEN_PIN, g > 0 ? HIGH : LOW);
  digitalWrite(BLUE_PIN,  b > 0 ? HIGH : LOW);
}
