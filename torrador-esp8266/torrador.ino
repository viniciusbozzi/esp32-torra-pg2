/***************************/
// Vinicius de Abreu Bozzi - TORRADOR DE CAFÉ
/***************************/

#if !( defined(ESP8266) ||  defined(ESP32) )
#error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif

// These definitions must be placed before #include <ESPAsync_WiFiManager.h>
#include "Async_ConfigOnDoubleReset_Multi.h"

#include <ESPAsync_WiFiManager.h>               //https://github.com/khoih-prog/ESPAsync_WiFiManager
#include <max6675.h>
#include <AsyncElegantOTA.h> //OTA
#include <PID_v1.h>

AsyncWebServer webServer(80);
DNSServer dnsServer;

//Customizacao do webserver
String myHostName = "My-ESP8266";
String myESP8266page = "<a href='/update'>Update Firmware</span></a>";
String myNotFoundPage = "<h2>Error, page not found! <a href='/'>Go back to main page!</a></h2>";
//static int PINOLED = 16;

//Definicoes do Sensor max6675
float temp_c;
int thermoSO = 23;
int thermoCS = 5;
int thermoSCK = 18;
MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);

const int fanPin = 22; //definicao rele ventilacao
const int pwmPin = 21;  //definicao do aquecedor (SSR VCC)

//definicao dados da torra
String dados_torra = "";
double *tempo, *temperatura;
boolean mostra_temp_app = false;
boolean iniciaDesligVent = false;

//Definicoes Firebase
#include <Firebase_ESP_Client.h>                //https://github.com/mobizt/Firebase-ESP-Client
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#define API_KEY ""
#define DATABASE_URL ""
#define USER_EMAIL ""
#define USER_PASSWORD ""
FirebaseData stream;
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
volatile bool configura_inicio = true;
volatile bool dataChanged = false;

unsigned long startedAtTemp = millis();
unsigned long startedAtTempVent = millis();
unsigned long startedAtTempAquec = millis();
boolean calculo_vetor_PID = true;

//Definicao variaveis PID
double Setpoint, Input, Output;
double Kp = 50, Ki = 1, Kd = 70; //default
unsigned long windowStartTime;
#define PWM_PERIOD 5000
#define FREQ_PID 250            // Run PID every 250 ms
unsigned long lastRun;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//variaveis de controle
volatile bool aquecendo = false;
volatile bool torrando = false;

//funçao que transforma o que e recebido pela serial em uma string
String lerserial() {
  String conteudo = ""; // armazena o frase inteira
  char carac; //armazena cada caracter
  while (Serial.available() > 0) { // enquando receber caracteres
    carac = Serial.read(); //grava-lo na variavel carac
    if (carac != '/n') { //se o caracter NAO for quebra de linha
      conteudo.concat(carac); //contatenar o caracter na string conteudo
    }
    delay(10); //delay para nova leitura
  }
  return conteudo; //retorna o valor da string conteudo
}

void separaTemperaturaETempo(double *v, int cont) {
  double *temper;
  temper = (double*) malloc(sizeof(double) * (cont / 2));
  double *temp;
  temp = (double*) malloc(sizeof(double) * (cont / 2));

  int j = 0, k = 0;
  for (int i = 0; i < (cont); i++) {
    if (i % 2 == 0) {
      temper[j] = v[i];
      j++;
    } else {
      temp[k] = v[i];
      k++;
    }
    temperatura = temper;
    tempo = temp;
  }
}


int conta_segundos = 0;
int posicao_segundos = 0;
double acumulado_segundos = 0.0;

void defineSetPoint() {

  int valor = digitalRead(fanPin);
  if (valor == HIGH) { //ventilaçao desligada
    Setpoint = 0.0;
    //reseta as variaveis de controle
    conta_segundos = 0;
    posicao_segundos = 0;
    acumulado_segundos = 0.0;
    calculo_vetor_PID = true;
  }
  if (valor == LOW && iniciaDesligVent) { //ventilação ligada porem resfriando
    Setpoint = 0.0;
    //reseta as variaveis de controle
    conta_segundos = 0;
    posicao_segundos = 0;
    acumulado_segundos = 0.0;
    torrando = false;
    aquecendo = false;
    calculo_vetor_PID = true;
  }

  if (valor == LOW && !iniciaDesligVent) { //aquecimento + ventilacao

    if (aquecendo) {
      if (temperatura[0] > 200) {
        Setpoint = 200;
      } else {
        Setpoint = temperatura[0];
      }
    }

    if (torrando) {
      double incremento = 0.0;

      if (conta_segundos == tempo[posicao_segundos]) {
        posicao_segundos++;
      }
      if (conta_segundos >= tempo[posicao_segundos - 1] && conta_segundos < tempo[posicao_segundos]) {
        incremento = (temperatura[posicao_segundos - 1] - temperatura[posicao_segundos]) / (tempo[posicao_segundos - 1] - tempo[posicao_segundos]);
      }
      conta_segundos++;
      acumulado_segundos = acumulado_segundos + incremento;

      if (temperatura[0] + acumulado_segundos > 200) {
        Setpoint = 200;
      } else {
        Setpoint = temperatura[0] + acumulado_segundos;
      }
    }
  }
}


//Funcao de callback do Firebase
void streamCallback(FirebaseStream data)
{

  yield();
  Serial.printf("sream path, %s\nevent path, %s\ndata type, %s\nevent type, %s\n\n",
                data.streamPath().c_str(),
                data.dataPath().c_str(),
                data.dataType().c_str(),
                data.eventType().c_str());
  printResult(data); // see addons/RTDBHelper.h
  Serial.println();

  // This is the size of stream payload received (current and max value)
  // Max payload size is the payload size under the stream path since the stream connected
  // and read once and will not update until stream reconnection takes place.
  // This max value will be zero as no payload received in case of ESP8266 which
  // BearSSL reserved Rx buffer size is less than the actual stream payload.
  Serial.printf("Received stream payload size: %d (Max. %d)\n\n", data.payloadLength(), data.maxPayloadLength());

  String mac = "/" + String(WiFi.macAddress());
  mac.toUpperCase();
  String aux2 = F("/aquecendo");
  String aux3 = mac + aux2; //dispositivos/MAC/aquecendo
  String aux4 = data.dataPath().c_str();
  String aux5 = F("/torrando");
  String aux6 = mac + aux5; //dispositivos/MAC/torrando
  String aux7 = F("/resfriando");
  String aux8 = mac + aux7;

  if ( aux4 == aux3 ) {
    yield();
    bool bVal;
    //dispositivos/MAC/aquecendo
    Serial.printf("Get bool ref... %s\n", Firebase.RTDB.getBool(&fbdo, data.streamPath().c_str() + aux4, &bVal) ? bVal ? "true" : "false" : fbdo.errorReason().c_str());
    yield();

    if (bVal) { //aquecendo = true
      aquecendo = true;
      mostra_temp_app = true;
      digitalWrite(fanPin, LOW);       // -> Liga motor DC
    } else { //aquecendo = false
      aquecendo = false;

      bool bVal2;
      //dispositivos/MAC/torrando
      yield();
      Serial.printf("Get bool ref... %s\n", Firebase.RTDB.getBool(&fbdo, data.streamPath().c_str() + aux6, &bVal2) ? bVal2 ? "true" : "false" : fbdo.errorReason().c_str());
      yield();
      if (bVal2) { //torrando = true
        torrando = true;
        digitalWrite(fanPin, LOW);
      } else {  //torrando = false
        torrando = false;
        iniciaDesligVent = true;
        startedAtTempVent = millis();
        mostra_temp_app = false;
        yield();
        Serial.printf("Set bool RESFRIANDO... %s\n", Firebase.RTDB.setBool(&fbdo,  data.streamPath().c_str() + aux8, true) ? "ok" : fbdo.errorReason().c_str());
        yield();
      }
    }
  }

  if ( aux4 == aux6 ) {
    yield();
    bool bVal2;
    //dispositivos/MAC/torrando
    Serial.printf("Get bool ref... %s\n", Firebase.RTDB.getBool(&fbdo, data.streamPath().c_str() + aux6, &bVal2) ? bVal2 ? "true" : "false" : fbdo.errorReason().c_str());
    yield();
    if (bVal2) { //torrando = true
      torrando = true;
      digitalWrite(fanPin, LOW);
    } else {   //torrando = false
      torrando = false;
      startedAtTempVent = millis();
      iniciaDesligVent = true;
      mostra_temp_app = false;
      yield();
      Serial.printf("Set bool RESFRIANDO... %s\n", Firebase.RTDB.setBool(&fbdo, data.streamPath().c_str() + aux8, true) ? "ok" : fbdo.errorReason().c_str());
      yield();
    }
  }

  // Due to limited of stack memory, do not perform any task that used large memory here especially starting connect to server.
  // Just set this flag and check it status later.
  dataChanged = true;
  yield();
}

void streamTimeoutCallback(bool timeout)
{
  yield();
  if (timeout)
    Serial.println("stream timed out, resuming...\n");

  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}


void setup()
{
  // put your setup code here, to run once:
  // initialize the LED digital pin as an output.
  pinMode(PIN_LED, OUTPUT);

  //RELÉ VENTILACAO
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, HIGH);

  //AQUECEDOR
  windowStartTime = millis();
  pinMode(pwmPin, OUTPUT);
  digitalWrite(pwmPin, LOW);

  // Init PID driver
  myPID.SetOutputLimits(0, 100);  // Give output as power percentage
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(FREQ_PID);
  myPID.SetTunings(Kp, Ki, Kd);


  Serial.begin(115200);
  while (!Serial);

  delay(200);

  Serial.print(F("\nStarting Async_ConfigOnDoubleReset_Multi using ")); Serial.print(FS_Name);
  Serial.print(F(" on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION);
  Serial.println(ESP_DOUBLE_RESET_DETECTOR_VERSION);

#if defined(ESP_ASYNC_WIFIMANAGER_VERSION_INT)
  if (ESP_ASYNC_WIFIMANAGER_VERSION_INT < ESP_ASYNC_WIFIMANAGER_VERSION_MIN)
  {
    Serial.print("Warning. Must use this example on Version later than : ");
    Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION_MIN_TARGET);
  }
#endif

  Serial.setDebugOutput(false);

  if (FORMAT_FILESYSTEM)
    FileFS.format();

  // Format FileFS if not yet
#ifdef ESP32
  if (!FileFS.begin(true))
#else
  if (!FileFS.begin())
#endif
  {
#ifdef ESP8266
    FileFS.format();
#endif

    Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));

    if (!FileFS.begin())
    {
      // prevents debug info from the library to hide err message.
      delay(100);

#if USE_LITTLEFS
      Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
#else
      Serial.println(F("SPIFFS failed!. Please use LittleFS or EEPROM. Stay forever"));
#endif

      while (true)
      {
        delay(1);
      }
    }
  }

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  unsigned long startedAt = millis();

  // New in v1.4.0
  initAPIPConfigStruct(WM_AP_IPconfig);
  initSTAIPConfigStruct(WM_STA_IPconfig);
  //////

  //Local intialization. Once its business is done, there is no need to keep it around
  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer);
  // Use this to personalize DHCP hostname (RFC952 conformed)
  //AsyncWebServer webServer(HTTP_PORT);

#if ( USING_ESP32_S2 || USING_ESP32_C3 )
  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, NULL, "AsyncConfigOnDoubleReset");
#else
  DNSServer dnsServer;

  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "CoffeESP");
#endif

  //ESPAsync_wifiManager.resetSettings();   //reset saved settings

#if USE_CUSTOM_AP_IP
  //set custom ip for portal
  // New in v1.4.0
  ESPAsync_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);
  //////
#endif

  ESPAsync_wifiManager.setMinimumSignalQuality(-1);

  // From v1.0.10 only
  // Set config portal channel, default = 1. Use 0 => random channel from 1-11
  ESPAsync_wifiManager.setConfigPortalChannel(0);
  //////

#if !USE_DHCP_IP
  // Set (static IP, Gateway, Subnetmask, DNS1 and DNS2) or (IP, Gateway, Subnetmask). New in v1.0.5
  // New in v1.4.0
  ESPAsync_wifiManager.setSTAStaticIPConfig(WM_STA_IPconfig);
  //////
#endif

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESPAsync_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif

  // We can't use WiFi.SSID() in ESP32 as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.println("ESP Self-Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  // SSID to uppercase
  ssid.toUpperCase();
  password   = "espcafe123";

  bool configDataLoaded = false;

  // From v1.1.0, Don't permit NULL password
  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());

    ESPAsync_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println(F("Got ESP Self-Stored Credentials. Timeout 120s for Config Portal"));
  }

  if (loadConfigData())
  {
    configDataLoaded = true;

    ESPAsync_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println(F("Got stored Credentials. Timeout 120s for Config Portal"));

#if USE_ESP_WIFIMANAGER_NTP
    if ( strlen(WM_config.TZ_Name) > 0 )
    {
      LOGERROR3(F("Current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

#if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org");
#else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
#endif
    }
    else
    {
      Serial.println(F("Current Timezone is not set. Enter Config Portal to set."));
    }
#endif
  }
  else
  {
    // Enter CP only if no stored SSID on flash and file
    Serial.println(F("Open Config Portal without Timeout: No stored Credentials."));
    initialConfig = true;
  }

  if (drd->detectDoubleReset())
  {
    // DRD, disable timeout.
    ESPAsync_wifiManager.setConfigPortalTimeout(0);

    Serial.println(F("Open Config Portal without Timeout: Double Reset Detected"));
    initialConfig = true;
  }

  if (initialConfig)
  {
    Serial.print(F("Starting configuration portal @ "));

#if USE_CUSTOM_AP_IP
    Serial.print(APStaticIP);
#else
    Serial.print(F("192.168.4.1"));
#endif

    Serial.print(F(", SSID = "));
    Serial.print(ssid);
    Serial.print(F(", PWD = "));
    Serial.println(password);

    digitalWrite(PIN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    //ESPAsync_wifiManager.setConfigPortalTimeout(600);

    // Starts an access point
    if (!ESPAsync_wifiManager.startConfigPortal((const char *) ssid.c_str(), password.c_str()))
      Serial.println(F("Not connected to WiFi but continuing anyway."));
    else
    {
      Serial.println(F("WiFi connected...yeey :)"));
    }

    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESPAsync_wifiManager.getSSID(i);
      String tempPW   = ESPAsync_wifiManager.getPW(i);

      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

#if USE_ESP_WIFIMANAGER_NTP
    String tempTZ   = ESPAsync_wifiManager.getTimezoneName();

    if (strlen(tempTZ.c_str()) < sizeof(WM_config.TZ_Name) - 1)
      strcpy(WM_config.TZ_Name, tempTZ.c_str());
    else
      strncpy(WM_config.TZ_Name, tempTZ.c_str(), sizeof(WM_config.TZ_Name) - 1);

    const char * TZ_Result = ESPAsync_wifiManager.getTZ(WM_config.TZ_Name);

    if (strlen(TZ_Result) < sizeof(WM_config.TZ) - 1)
      strcpy(WM_config.TZ, TZ_Result);
    else
      strncpy(WM_config.TZ, TZ_Result, sizeof(WM_config.TZ_Name) - 1);

    if ( strlen(WM_config.TZ_Name) > 0 )
    {
      LOGERROR3(F("Saving current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

#if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org");
#else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
#endif
    }
    else
    {
      LOGERROR(F("Current Timezone Name is not set. Enter Config Portal to set."));
    }
#endif

    // New in v1.4.0
    ESPAsync_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
    //////

    saveConfigData();
  }

  digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.

  startedAt = millis();

  if (!initialConfig)
  {
    // Load stored data, the addAP ready for MultiWiFi reconnection
    if (!configDataLoaded)
      loadConfigData();

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    if ( WiFi.status() != WL_CONNECTED )
    {
      Serial.println(F("ConnectMultiWiFi in setup"));

      connectMultiWiFi();
    }
  }

  Serial.print(F("After waiting "));
  Serial.print((float) (millis() - startedAt) / 1000);
  Serial.print(F(" secs more in setup(), connection result is "));

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print(F("connected. Local IP: "));
    Serial.println(WiFi.localIP());
  }
  else
    Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));


  //FOTAELEGANT
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(200, "text/html", myESP8266page);
  });
  webServer.onNotFound([](AsyncWebServerRequest * request)
  {
    request->send(404, "text/html", myNotFoundPage);
  });

  AsyncElegantOTA.begin(&webServer, "admin", "espcafe123"); // Start ElegantOTA
  //AsyncElegantOTA.begin(&webServer); // Start ElegantOTA
  webServer.begin();
  Serial.println("FOTA server ready!");
  //pinMode(PINOLED, OUTPUT);

  //Confirmacao ESP-APP
  webServer.on("/confirm.html", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(200, "text/html", "<title>torradeira</title>");
  });



  //FIREBASE
  Serial.println();
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  Firebase.begin(&config, &auth);

  Firebase.reconnectWiFi(true);

  // Recommend for ESP8266 stream, adjust the buffer size to match your stream data size
#if defined(ESP8266)
  stream.setBSSLBufferSize(2048 /* Rx in bytes, 512 - 16384 */, 512 /* Tx in bytes, 512 - 16384 */);
#endif

  delay(10);
  if (!Firebase.RTDB.beginStream(&stream, "/dispositivos"))
    Serial.printf("sream begin error, %s\n\n", stream.errorReason().c_str());

  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);

  //ESP.wdtDisable();

}

void runPID() {
  unsigned long now = millis();

  Input = thermocouple.readCelsius();

  if (!isnan(Input)) { //not NaN
    myPID.Compute();
  }

  if (now - windowStartTime > PWM_PERIOD)
  {
    windowStartTime += PWM_PERIOD;
  }

  Serial.print("SETPOINT: ");
  Serial.println(Setpoint);
  Serial.print("INPUT: ");
  Serial.println(Input);
  Serial.print("OUTPUT: ");
  Serial.println(Output);

  // Simulate low frequency PWM
  if ((Output * PWM_PERIOD) / 100 > now - windowStartTime)
  {
    digitalWrite(pwmPin, HIGH);
    Serial.println("Relé alto");
  }
  else
  {
    digitalWrite(pwmPin, LOW);
    Serial.println("Relé baixo");
  }
}


void loop()
{
  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  yield();
  drd->loop();

  if (millis() - startedAtTemp > 3000 || startedAtTemp == 0)
  {
    startedAtTemp = millis();
    temp_c = thermocouple.readCelsius();
    Serial.print("t1=");
    Serial.println(temp_c);

    if (mostra_temp_app) {
      String aux = "/dispositivos/";
      String mac = String(WiFi.macAddress());
      mac.toUpperCase();
      String aux2 = "/temperatura";
      String aux3 = aux + mac + aux2;
      yield();
      if (!isnan(Input)) {
        Serial.printf("Set float... %s\n", Firebase.RTDB.setFloat(&fbdo, aux3 , temp_c) ? "ok" : fbdo.errorReason().c_str());
      }
      yield();
    }
  }

  if (millis() - startedAtTempAquec > 1000 || startedAtTempAquec == 0) {
    startedAtTempAquec = millis();

    if (mostra_temp_app && calculo_vetor_PID) {
      String aux = "/dispositivos/";
      String mac = String(WiFi.macAddress());
      mac.toUpperCase();
      String aux2 = "/dados";
      String aux3 = aux + mac + aux2;
      yield();
      Serial.printf("Get string... %s\n", Firebase.RTDB.getString(&fbdo, aux3) ? fbdo.to<const char *>() : fbdo.errorReason().c_str());
      yield();
      String aux4 = "A,";
      String aux5 = (fbdo.to<const char *>());
      dados_torra = aux4 + aux5;
      yield();

      char c[1000];
      dados_torra.toCharArray(c, 1000);

      int tamanho = dados_torra.length();
      int quantidade = 0;

      for (int i = 0; i < tamanho; i++) {
        if (c[i] == ',') {
          Serial.println(c[i]);
          quantidade++;
        }
      }
      
      char* token = strtok(c, ",");
      double v[(quantidade * 2) - 1];
      for (int i = 0; i < quantidade; i++) {
        token = strtok(NULL, ",");
        v[i] = strtod(token, NULL);
        //Serial.println(v[i]);
      }

      separaTemperaturaETempo(v, quantidade);
      v[(quantidade * 2) - 1] = NULL;

      calculo_vetor_PID = false;
    }
    
    defineSetPoint();
  }


  if (millis() - startedAtTempVent > 10000 && iniciaDesligVent) { // 100 segundos de ventilação ligada após termino da torra

    String aux = "/dispositivos/";
    String mac = String(WiFi.macAddress());
    mac.toUpperCase();
    String aux2 = "/resfriando";
    String aux3 = aux + mac + aux2;
    yield();
    Serial.printf("Set bool... %s\n", Firebase.RTDB.setBool(&fbdo, aux3, false) ? "ok" : fbdo.errorReason().c_str());
    yield();

    digitalWrite(fanPin, HIGH);
    iniciaDesligVent = false;
  }


  if (Firebase.ready() && configura_inicio)
  {
    String path_aux = "/dispositivos/";
    String mac = String(WiFi.macAddress());
    String ip = WiFi.localIP().toString();

    FirebaseJson json2;
    json2.add(mac);

    yield();
    Serial.printf("Set string... %s\n", Firebase.RTDB.updateNodeAsync(&fbdo, path_aux, &json2) ? "ok" : fbdo.errorReason().c_str());
    yield();

    FirebaseJson json;
    json.add("ip", ip);
    json.add("temperatura", 20.0);
    json.add("torrando", false);
    json.add("dados", "50.0,0.0,200.0,120.0");
    json.add("aquecendo", false);
    json.add("resfriando", false);
    yield();
    Serial.printf("Set json... %s\n\n", Firebase.RTDB.updateNodeAsync(&fbdo, path_aux + mac, &json) ? "ok" : fbdo.errorReason().c_str());
    yield();

    configura_inicio = false;
  }

  if (dataChanged)
  {
    dataChanged = false;
    // When stream data is available, do anything here...
  }

  if (Serial.available()) { // se estiver recebendo algo pela serial
    String rec = lerserial(); //executar a funçao que transforma os caracteres em string e grava a saida na var rec
    char carc = rec[0];
    Serial.println(carc);
    if (carc == 'S') {
      rec.remove(0, 1);
      Serial.println(rec);
      Setpoint = rec.toDouble();
    }
    if (carc == 'P') {
      rec.remove(0, 1);
      Kp = rec.toDouble();
    }
    if (carc == 'I') {
      rec.remove(0, 1);
      Ki = rec.toDouble();
    }
    if (carc == 'D') {
      rec.remove(0, 1);
      Kd = rec.toDouble();
    }
  }

  unsigned long now = millis();
  unsigned long elapsed = (now - lastRun);
  if (elapsed >= FREQ_PID)
  {
    runPID();
    lastRun = now;
  }

  // put your main code here, to run repeatedly
  check_status();
  yield();
}
