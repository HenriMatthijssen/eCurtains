// =========================================================================================
// 
// eCurtains v1.07: Software for controlling your curtains with a stepper motor using
//                  Arduino ESP8266, '28BYJ-48 Stepper motor and ULN Stepper Motor Driver'
//                  or 'Nema17 Stepper Motor and A4988 Stepper Motor Driver'.
//
//                  For faster stepper motor performance I recommend to use the Nema17
//                  Stepper Motor.
//
//                  See 'Blog Henri Matthijssen' for instructions for the hardware and setup
//                  http://matthijsseninfo.nl
//
//                  (c) 2018 Henri Matthijssen (henri@matthijsseninfo.nl)
//
//                  Please do not distribute without permission of the original author and 
//                  respect his time and work spent on this.
//
// =========================================================================================
//
// Please follow instructions below to compile this code in the Arduino IDE:
//
// Please make sure that you have set next line in 'Additional Board Manager URL's' of 
// Preferences:
//
// http://arduino.esp8266.com/stable/package_esp8266com_index.json
//
// Then goto Boards Manager (‘Tools > Board’) and install the ‘esp8266 by ESP8266 Community’ entry.
// Finally set your board to 'NodeMCU 1.0 (ESP-12E Module)' in Tools menu.
//
// Further make sure that in the Libraries Manager (Sketch > Include Library > Manage Libraries...'
// you have installed the 'AccelStepper' library from Mike McCauley.
//
// =========================================================================================
// 
// First time you installed the software the ESP8266 will startup in Access-Point (AP) mode.
// Connect with you WiFi to this AP using next information:
// 
// AP ssid           : eCurtains
// AP password       : eCurtains
//
// Enter IP-address 192.168.4.1 in your web-browser. This will present you with dialog to fill
// the credentials of your home WiFi Network. Fill your home SSID and password and press the
// submit button. Your ESP8266 will reboot and connect automatically to your home WiFi Network.
// Now find the assigned IP-address to the ESP8266 and use that again in your web-browser.
//
// Before you can use the Web-Interface you have to login with a valid user-name
// and password on location: http://ip-address/login (cookie is valid for 24 hours).
//
// Default user-name : admin
// Default password  : notdodo
//
// In the Web-Interface you can change the default user-name / password (among other
// settings like language, host-name, step motor type & driver, motor acceleration speed 
// and motor maximum speed).
// 
// First action you have to do in the software is calibrating your curtains by setting
// the 'left' (open) and 'right' (closed) position of your curtains. Either use the
// Web-Interface or the API for this.
//
// After calibration you can:
// 
// Totally open your curtains by:
//
// - pressing [Move Total Left] button in the Web-Interface
//   or
// - calling API with: http://ip-address/api?action=move_total_left&api=your_api
//
// Totally close your curtains by:
//
// - pressing [Move Total Right] button in the Web-Interface
//   or
// - calling API with: http://ip-address/api?action=move_total_right&api=your_api
//
// When using the API you always need to supply a valid value for the API key
// (default value=27031969). The API has next format in the URL:
//
// http://ip-address/api?action=xxx&value=yyy&api=zzz
//
// Currently next API actions are supported:
//
// reboot, value              (value=false or true)
// reset, value               (value=false or true)
//
// set_api, value             (value=new API key, can only be changed once via API)
// set_host, value            (value=new host-name)
// set_language, value        (0=English, 1=Dutch (default))
//
// reset_left
// reset_current
// reset_right
//
// set_left, [value]          (if no value supplied then current position is used)
// move_motor, value
// set_right, [value]         (if no value supplied then current position is used)
//
// stop_motor
//
// motor_acceleration, value  (value=acceleration)
// motor_max_speed, value     (value=maximum speed)
//
// move_total_left
// move_total_right
//
// You can upgrade the firmware with the (hidden) firmware upgrade URL:
// http://ip-address/upgradefw
// (you first needed to be logged in for the above URL to work)
//
// You can erase all settings by clearing the EEPROM with next URL:
// http://ip-address/erase
// (you first needed to be logged in for the above URL to work)
//
// =========================================================================================

// Current version of the eCurtains software
float g_current_version = 1.07;

// -----------------------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------------------

// For ESP8266
#include <ESP8266WiFi.h>                            // ESP8266 WiFi
#include <ESP8266WebServer.h>                       // ESP8266 Web Server
#include <EEPROM.h>                                 // EEPROM
#include <SPI.h>                                    // Serial Peripheral Interrface
#include <FS.h>                                     // File system

// For Steppermotor
#include <AccelStepper.h>                           // Acceleration Stepper Driver

// -----------------------------------------------------------------------------------------
// DEFINES
// -----------------------------------------------------------------------------------------

// Steppermotor settings
#define STEPS_PER_REVOLUTION_28BYJ           2048   // Steps per Revolution
#define STEPS_PER_REVOLUTION_NEMA17           200   // Steps per Revolution

#define MOTOR_ACCELERATION                  100.0   // Acceleration of motor
#define MOTOR_MAX_SPEED                     250.0   // Maximum speed of motor

#define DEFAULT_POSITION_LEFT              -100.0   // Default Treshold Left
#define DEFAULT_POSITION_CURRENT_PREVIOUS     0.0   // Default Previous Position
#define DEFAULT_POSITION_CURRENT              0.0   // Default Current Position
#define DEFAULT_POSITION_RIGHT              100.0   // Default Treshold Right

// Defines for EEPROM map
#define LENGTH_SSID                            32   // Maximum length SSID
#define LENGTH_PASSWORD                        64   // Maximum length Password for SSID
#define LENGTH_HOSTNAME                        15   // Maximum length hostname
#define LENGTH_API_KEY                         64   // Maximum length API Key

#define LENGTH_WEB_USER                        64   // Maximum lenght of web-user
#define LENGTH_WEB_PASSWORD                    64   // Maximum lenght of web-password

#define ALLOCATED_EEPROM_BLOCK               1024   // Size of EEPROM block that you claim

//
// Defines for used pins on ESP8266 for controlling motor
//
// Remark: for NodeMCU the hard-coded pin-numbers can be found in file
// \variants\nodemcu\pins_arduino
//
// In this file you will find:
//
//  static const uint8_t D0   = 16;
//  static const uint8_t D1   = 5;
//  static const uint8_t D2   = 4;
//  static const uint8_t D3   = 0;
//  static const uint8_t D4   = 2;
//  static const uint8_t D5   = 14;
//  static const uint8_t D6   = 12;
//  static const uint8_t D7   = 13;
//  static const uint8_t D8   = 15;
//  static const uint8_t D9   = 3;
//  static const uint8_t D10  = 1;
//
#define MOTOR_PIN1               D1       // ULN2003A Driver: IN1 pin
                                          // or
                                          // A4988 Driver: DIRECTION pin
                                          
#define MOTOR_PIN2               D2       // ULN2003A Driver: IN2 pin
                                          // or
                                          // A4988 Driver: STEPS pin
                                          
#define MOTOR_PIN3               D5       // ULN2003A Driver: IN3 pin
                                          // or
                                          // A4988 Driver: SLEEP pin
                                          
#define MOTOR_PIN4               D6       // ULN2003A Driver: IN4 pin
                                          // or
                                          // A4988 Driver: RESET pin

#define BUILTIN_LED1             D4       // Builtin LED of ESP8266 is on GPIO2 (=D4)

// Miscellaneous defines
#define g_epsilon           0.00001       // Used for comparing fractional values

// Default language
#define DEFAULT_LANGUAGE          1       // Default language is Dutch (=1), 0=English

// -----------------------------------------------------------------------------------------
// CONSTANTS
// -----------------------------------------------------------------------------------------

// ESP8266 settings
const char*       g_AP_ssid              = "eCurtains";  // Default Access Point ssid
const char*       g_AP_password          = "eCurtains";  // Default Password for AP ssid

const char*       g_default_host_name    = "eCurtains";  // Default Hostname
const char*       g_default_api_key      = "27031969";   // Default API key

const char*       g_default_web_user     = "admin";      // Default Web User
const char*       g_default_web_password = "notdodo";    // Default Web Password

// -----------------------------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------------------------

// WiFiClient & Webserver
WiFiClient        serverClients[5];                      // Maximum number of Server Clients
ESP8266WebServer  server(80);                            // Listen port Webserver

unsigned int      g_start_eeprom_address = 0;            // Start offset in EEPROM


// Structure used for EEPROM read/write
struct {
  
  char  ssid           [ LENGTH_SSID      + 1 ]     = "";
  char  password       [ LENGTH_PASSWORD  + 1 ]     = "";

  float version                                     = 0.0;

  char  hostname       [ LENGTH_HOSTNAME  + 1 ]     = "";
  char  apikey         [ LENGTH_API_KEY   + 1 ]     = "";
  int   apikey_set                                  = 0;

  float position_left                               = DEFAULT_POSITION_LEFT;
  float position_current_previous                   = DEFAULT_POSITION_CURRENT_PREVIOUS;
  float position_current                            = DEFAULT_POSITION_CURRENT;
  float position_right                              = DEFAULT_POSITION_RIGHT;

  int   language                                    = DEFAULT_LANGUAGE;

  char  web_user       [ LENGTH_WEB_USER     + 1 ]  = "";
  char  web_password   [ LENGTH_WEB_PASSWORD + 1 ]  = "";

  float motor_acceleration                          = MOTOR_ACCELERATION;
  float motor_max_speed                             = MOTOR_MAX_SPEED;
  float steps_per_revolution                        = STEPS_PER_REVOLUTION_28BYJ;
  int   stepper_motor_and_driver                    = 0;      // 0 = 28BYJ-48 + ULN, 1 = Nema17 + A4988
  
} g_data;

// For upgrading firmware
File    					g_upload_file;
String  					g_filename;

// Running in Access Point (AP) Mode (=0) or Wireless Mode (=1). Default start with AP Mode
int     					g_wifi_mode         = 0;

// Global variable for using AccelStepper (default is 28BYJ-48 Stepper Motor)
AccelStepper  		g_AccelStepper;

// -----------------------------------------------------------------------------------------
// LANGUAGE STRINGS
// -----------------------------------------------------------------------------------------
const String l_configure_wifi_network[] = {
    "Configure WiFi Network",
    "Configureer WiFi Netwerk",
  };
const String l_enter_wifi_credentials[] = {
    "Provide Network SSID and password",
    "Vul Netwerk SSID en wachtwoord in",
  };
const String l_network_ssid_label[] = {
    "Network SSID       : ",
    "Netwerk SSID       : ",
  };
const String l_network_password_label[] = {
    "Network Password   : ",
    "Netwerk Wachtwoord : ",
  };
  
const String l_network_ssid_hint[] = {
    "network SSID",
    "netwerk SSID",
  };
const String l_network_password_hint[] = {
    "network password",
    "netwerk wachtwoord",  
  };

const String l_login_screen[] = {
    "eCurtains Login", 
    "eCurtains Login" ,
  };
const String l_provide_credentials[] = { 
    "Please provide a valid user-name and password", 
    "Geef een correcte gebruikersnaam en wachtwoord" ,
  };
const String l_user_label[] = {
    "User       : ",
    "Gebruiker  : ", 
  };
const String l_password_label[] = {
    "Password   : ",
    "Wachtwoord : ",
  };  
  
const String l_username[] = {
    "user-name",
    "gebruikersnaam",
  };
const String l_password[] = {
    "password",
    "wachtwoord",
  };

const String l_settings_screen[] = {
    "eCurtains Settings", 
    "eCurtains Instellingen",
  };
const String l_provide_new_credentials[] = { 
    "Please provide a new user-name, password and API key", 
    "Geef een nieuwe gebruikersnaam, wachtwoord en API key",
  };
const String l_new_user_label[] = {
    "New User          : ",
    "Nieuwe Gebruiker  : ", 
  };
const String l_new_password_label[] = {
    "New Password      : ",
    "Nieuw Wachtwoord  : ",
  };  
const String l_new_api_key[] = {
    "New API Key       : ",
    "Nieuwe API Key    : ",
  };
  
const String l_change_language_hostname[] = {
    "Change language and Hostname",
    "Verander taal en hostnaam"
  };
const String l_language_label[] = {
    "New Language      : ",
    "Nieuwe Taal       : ",
  };
const String l_hostname_label[] = {
    "New Hostname      : ",
    "Nieuwe Hostnaam   : ",
  };
const String l_language_english[] = {
    "English",
    "Engels",
  };
const String l_language_dutch[] = {
    "Dutch",
    "Nederlands",
  };
const String l_change_motor_settings[] = {
    "Change motor settings",
    "Verander motor instellingen",
  };
const String l_change_step_motor_and_driver[] = {
    "Stepper Motor + Driver  : ",
    "Stappen Motor + Driver  : ",
  };
const String l_28BYJ_ULN[] = {
    "28BYJ-48 + ULN",
    "28BYJ-48 + ULN",
  };
const String l_Nema17_A4988[] = {
    "Nema17 + A4988",
    "Nema17 + A4988",
  }; 
const String l_change_steps_per_revolution[] = {
    "Steps per Revolution    : ",
    "Stappen per Omwenteling : ",
  };
const String l_change_motor_acceleration[] = {
    "Acceleration            : ",
    "Versnelling             : ",
  };
const String l_change_motor_max_speed[] = {
    "Maximum speed           : ",
    "Maximale snelheid       : ",
  };

const String l_login_again [] = {
    "After pressing [Submit] you possibly need to login again",
    "Na het drukken van [Submit] moet je mogelijk opnieuw inloggen",
  };

const String l_version[] = {
    "Version",
    "Versie",
  };

const String l_hostname[] = {
    "Hostname ",
    "Hostnaam ",
  };

const String l_btn_logout[] = {
    "Logout",
    "Uitloggen",
  };
const String l_btn_settings[] = {
    "Settings",
    "Instellingen",
  };

const String l_overview_header[] = {
  "Overview",
  "Overzicht",
  };
const String l_overview_1[] = {
    "      |                v                   |     ",
    "      |                v                   |     "
  };
const String l_overview_2[] = {
    " -----+------------------------------------+-----",
    " -----+------------------------------------+-----",
  };
const String l_overview_3[] = {
    "   treshold         current            treshold  ",
    "   drempel          huidige             drempel  ", 
  };
const String l_overview_4[] = {
    "   left             position              right  ",
    "   links            positie              rechts  ",
  };
const String l_overview_5[] = {
    "You cannot move the motor outside the thresholds.",
    "Je kunt de motor niet buiten de drempel waarden bewegen.",
  };
  
const String l_positions[] = { 
    "Positions", 
    "Posities" ,
  };
const String l_treshold_left [] = {
    "Treshold Left : ",
    "Drempel Links : ",
  };
const String l_previous_position [] = {
    "Previous Position : ",
    "Vorige Positie : ",
  };
const String l_current_position [] = {
    "Current Position : ",
    "Huidige Positie : ",
  };
const String l_treshold_right [] = {
    "Treshold Right : ",
    "Drempel Rechts : ",
  };

const String l_btn_treshold_left [] = {
    "Treshold Left",
    "Drempel Links",
  };
const String l_btn_current_position [] = {
    "Current Position",
    "Huidige Positie",
  };
const String l_btn_treshold_right [] = {
    "Treshold Right",
    "Drempel Rechts",
  };

const String l_move_motor [] = {
    "Move Motor",
    "Beweeg Moter",
  };
  
const String l_btn_left_100 [] = {
    "Left -100",
    "Links -100",
  };

const String l_btn_left_10 [] = {
    "Left -10",
    "Links -10",
  };
const String l_btn_left_1 [] = {
    "Left -1",
    "Links -1",
  };
const String l_btn_left_0_5 [] = {
    "Left -0.5",
    "Links -0.5",
  };
const String l_btn_left_0_25 [] = {
    "Left -0.25",
    "Links -0.25",
  };
const String l_btn_left_0_1 [] = {
    "Left -0.1",
    "Links -0.1",
  };
const String l_stop_motor [] = {
    "Stop Motor",
    "Stop Moter",
  };
const String l_btn_right_0_1 [] = {
    "Right 0.1",
    "Rechts 0.1",
  };
const String l_btn_right_0_25 [] = {
    "Right 0.25",
    "Rechts 0.25",
  };
const String l_btn_right_0_5 [] = {
    "Right 0.5",
    "Rechts 0.5",
  };
const String l_btn_right_1 [] = {
    "Right 1",
    "Rechts 1",
  };
const String l_btn_right_10 [] = {
    "Right 10",
    "Rechts 10",
  };
const String l_btn_right_100 [] = {
    "Right 100",
    "Rechts 100",
  };

const String l_set_new_tresholds [] = {
    "Set Treshold to current Position",
    "Stel drempel waarde in met huidige Positie",
  };

const String l_move_to_tresholds [] = {
    "Move Motor to Treshold",
    "Beweeg Moter naar drempel waarde",
  };

const String l_btn_move_total_left [] = {
    "Total Left",
    "Helemaal Links",
  };
const String l_btn_move_total_right [] = {
    "Total Right",
    "Helemaal Rechts",
  };

const String l_after_upgrade [] = {
    "After pressing 'Upgrade' please be patient. Wait until device is rebooted.",
    "Wees geduldig na het drukken van 'Upgrade'. Wacht totdat het device opnieuw is opgestart.",
  };

// -----------------------------------------------------------------------------------------
// IMPLEMENTATION OF METHODS    
// -----------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------
// setup
// Default setup() method called once by Arduino
// -----------------------------------------------------------------------------------------
void setup() 
{
  // Setup serial port for your Serial Monitor
  Serial.begin(115200);

  // Initialize EEPROM before you can use it 
  // (should be enough to fit g_data structure)
  EEPROM.begin( ALLOCATED_EEPROM_BLOCK );

  // Read current contents of EEPROM
  EEPROM.get ( g_start_eeprom_address, g_data );

  // Set internal LED as Output
  pinMode(BUILTIN_LED1, OUTPUT);

  // Switch off internal LED
  digitalWrite(BUILTIN_LED1, HIGH);

  delay(200);

  Serial.println("");
  Serial.println("");
  Serial.println("Initializing");
  Serial.println("");

  // If current major version differs from EEPROM, write back new default values
  if ( (int)g_data.version != (int)g_current_version )
  {
    Serial.print("Major version in EEPROM ('");
    Serial.print(g_data.version);
    Serial.print("') differs with this current version ('");
    Serial.print(g_current_version);
    Serial.println("')"), 
    Serial.println("Setting default values in EEPROM.");
    Serial.println("");

    // Write back new default values
    strcpy (g_data.ssid,               "");
    strcpy (g_data.password,           "");
    
    strcpy (g_data.hostname,           g_default_host_name);
    g_data.version                   = g_current_version;
    strcpy (g_data.apikey,             g_default_api_key);
    g_data.apikey_set                = 0;

    strcpy(g_data.web_user,            g_default_web_user);
    strcpy(g_data.web_password,        g_default_web_password);
    
    g_data.position_left             = DEFAULT_POSITION_LEFT;
    g_data.position_current_previous = DEFAULT_POSITION_CURRENT_PREVIOUS;
    g_data.position_current          = DEFAULT_POSITION_CURRENT;
    g_data.position_right            = DEFAULT_POSITION_RIGHT;

    g_data.language                  = DEFAULT_LANGUAGE;   

    g_data.motor_acceleration        = MOTOR_ACCELERATION;
    g_data.motor_max_speed           = MOTOR_MAX_SPEED;    
  }

  // Set current version
  g_data.version = g_current_version;
  
  // Store values into EEPROM
  EEPROM.put( g_start_eeprom_address, g_data );
  EEPROM.commit();

  // Display current set hostname
  Serial.print("EEPROM Hostname          : ");
  Serial.println(g_data.hostname);

  // Set Hostname
  WiFi.hostname( g_data.hostname );    

  // Display current 'eCurtains' version
  Serial.print("EEPROM eCurtains version : ");
  Serial.println(g_data.version);

  // Display current Web-Page Language Setting
  Serial.print("EEPROM Language          : ");

  if (g_data.language == 0)
  {
    Serial.println(l_language_english[g_data.language]);
  }
  else
  {
    Serial.println(l_language_dutch[g_data.language]);
  }
  
  // Display current API key
  Serial.print("EEPROM API key           : ");
  Serial.println(g_data.apikey);
  Serial.print("EEPROM API key set       : ");
  Serial.println(g_data.apikey_set);

  Serial.println("");

  // Display set web-user & web-password
  Serial.print("EEPROM Web User          : ");
  Serial.println(g_data.web_user);
  Serial.print("EEPROM Web Password      : ");
  Serial.println(g_data.web_password);
  Serial.println("");

  Serial.print("EEPROM Motor + driver    : ");
  if (g_data.stepper_motor_and_driver == 0)
  {
    Serial.println(l_28BYJ_ULN[g_data.language]);
  }
  else
  {
    Serial.println(l_Nema17_A4988[g_data.language]);
  }
  Serial.print("EEPROM Steps/Revolution  : ");
  Serial.println(g_data.steps_per_revolution);
  Serial.print("EEPROM Acceleration      : ");
  Serial.println(g_data.motor_acceleration);
  Serial.print("EEPROM Max Speed         : ");
  Serial.println(g_data.motor_max_speed);
  Serial.println("");
  
  // Display current set positions
  Serial.print("EEPROM Position (left)   : ");
  Serial.println(g_data.position_left);
  Serial.print("EEPROM Position (prv)    : ");
  Serial.println(g_data.position_current_previous);
  Serial.print("EEPROM Position (cur)    : ");
  Serial.println(g_data.position_current);
  Serial.print("EEPROM Position (right)  : ");
  Serial.println(g_data.position_right);

  Serial.println("");

  // Display stored SSID & password
  Serial.print("EEPROM ssid              : ");
  Serial.println(g_data.ssid);
  Serial.print("EEPROM password          : ");
  Serial.println(g_data.password);

  Serial.println("");

  // Check if SPIFFS is OK
  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS failed, needs formatting");

    handle_format();
    delay(200);

    ESP.restart();
  }

  // AP or WiFi Connect Mode

  // No WiFi configuration, then default start with AP mode
  if ( (strlen(g_data.ssid) == 0) || (strlen(g_data.password) == 0) )
  {
    Serial.println("No WiFi configuration found, starting in AP mode");
    Serial.println("");

    Serial.print("SSID          : '");
    Serial.print(g_AP_ssid);
    Serial.println("'");
    Serial.print("Password      : '");
    Serial.print(g_AP_password);
    Serial.println("'");

    // Ask for WiFi network to connect to and password
    g_wifi_mode = 0;
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(g_AP_ssid, g_AP_password);

    Serial.println("");
    Serial.print("Connected to  : '");
    Serial.print(g_AP_ssid);
    Serial.println("'");
    Serial.print("IP address    : ");
    Serial.println(WiFi.softAPIP());
  }
  // WiFi configuration found, try to connect
  else
  {
    int i = 0;

    Serial.print("Connecting to : '");
    Serial.print(g_data.ssid);
    Serial.println("'");

    WiFi.mode(WIFI_STA);
    WiFi.begin(g_data.ssid, g_data.password);

    while (WiFi.status() != WL_CONNECTED && i < 31)
    {
      delay(1000);
      Serial.print(".");
      ++i;
    }

    // Unable to connect to WiFi network with current settings
    if (WiFi.status() != WL_CONNECTED && i >= 30)
    {
      WiFi.disconnect();

      g_wifi_mode = 0;

      delay(1000);
      Serial.println("");

      Serial.println("Couldn't connect to network :( ");
      Serial.println("Setting up access point");
      Serial.print("SSID     : '");
      Serial.print(g_AP_ssid);
      Serial.println("'");
      Serial.print("Password : ");
      Serial.println(g_AP_password);
      
      // Ask for WiFi network to connect to and password
      WiFi.mode(WIFI_AP);
      WiFi.softAP(g_AP_ssid, g_AP_password);
      
      Serial.print("Connected to  : '");
      Serial.println(g_AP_ssid);
      Serial.println("'");
      IPAddress myIP = WiFi.softAPIP();
      Serial.print("IP address    : ");
      Serial.println(myIP);
    }
    // Normal connecting to WiFi network
    else
    {
      g_wifi_mode = 1;

      Serial.println("");
      Serial.print("Connected to  : '");
      Serial.print(g_data.ssid);
      Serial.println("'");
      Serial.print("IP address    : ");
      Serial.println(WiFi.localIP());

      // Disable sleep Mode
      // WiFi.setSleepMode(WIFI_NONE_SLEEP);      
    }
  }

  // If connected to AP mode, display simple WiFi settings page
  if ( g_wifi_mode == 0)
  {
      Serial.println("");
      Serial.println("Connected in AP mode thus display simple WiFi settings page");
      server.on("/", handle_wifi_html);
  }
  // If connected in WiFi mode, display the eCurtains Web-Interface
  else
  {
    Serial.println("");
    Serial.println("Connected in WiFi mode thus display eCurtains Web-Interface");
    server.on ( "/", handle_root ); 
  }

  // Setup supported URL's for this ESP8266
  server.on("/format", handle_format );                     // Format SPIFFS
  server.on("/erase", handle_erase );                       // Erase EEPROM
  server.on("/api", handle_api);                            // API of eCurtains

  server.on("/wifi_ajax", handle_wifi_ajax);                // Handle simple WiFi dialog
  
  server.on("/login", handle_login_html);                   // Login Screen
  server.on("/login_ajax", handle_login_ajax);              // Handle Login Screen

  server.on("/settings", handle_settings_html);             // Change Settings
  server.on("/settings_ajax", handle_settings_ajax);        // Handle Change Settings

  server.on("/upgradefw", handle_upgradefw_html);           // Upgrade firmware
  server.on("/upgradefw2", HTTP_POST, []() 
  {
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() 
  {
    HTTPUpload& upload = server.upload();
    
    if (upload.status == UPLOAD_FILE_START)
    {
      g_filename = upload.filename;
      Serial.setDebugOutput(true);
      
      Serial.printf("Starting upgrade with filename: %s\n", upload.filename.c_str());
      Serial.printf("Please be patient...\n");
      
      uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

      // Start with max available size
      if ( !Update.begin( maxSketchSpace) ) 
      { 
          Update.printError(Serial);
      }
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
        {
          Update.printError(Serial);
        }
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
        if (Update.end(true)) //true to set the size to the current progress
        {
          Serial.printf("Upgrade Success: %u\nRebooting...\n", upload.totalSize);
        }
        else
        {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
    }
    yield();
  });

  // List of headers to be recorded  
  const char * headerkeys[] = {"User-Agent","Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys)/sizeof(char*);

  // Ask server to track these headers
  server.collectHeaders(headerkeys, headerkeyssize );

  // Initialize AccelStepper library:
  if (g_data.stepper_motor_and_driver == 0)
  {  
    // 28BYJ-48 + ULN
    // Remark: use pin sequence: IN1-IN3-IN2-IN4 when using the AccelStepper with 28BYJ-48    
    g_AccelStepper = AccelStepper( AccelStepper::FULL4WIRE, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4 );

    Serial.println();
    Serial.println("Initializing AccelStepper for 28BYJ-48 + ULN");
  }
  else
  {
    // Nema17 + A4988
    // Remark: connect MOTOR_PIN1 to 'direction' and MOTOR_PIN2 to 'steps' of A4988 Driver
    g_AccelStepper = AccelStepper( AccelStepper::DRIVER, MOTOR_PIN2, MOTOR_PIN1 );

    // Set 'reset' pin to HIGH
    pinMode( MOTOR_PIN4, OUTPUT);
    digitalWrite( MOTOR_PIN4, HIGH );
    
    Serial.println();
    Serial.println("Initializing AccelStepper for Nema17 + A4988");
  }
  
  // Initialize global speed settings for the Motor Driver 

  // Set acceleration
  g_AccelStepper.setAcceleration ( g_data.motor_acceleration );

  // Set maximum motor speed
  g_AccelStepper.setMaxSpeed ( g_data.motor_max_speed );

  Serial.println("");

  // Start HTTP server
  server.begin();
  Serial.println("HTTP server started");  
}

// -----------------------------------------------------------------------------------------
// loop ()
// Here you find the main code which is run repeatedly
// -----------------------------------------------------------------------------------------
void loop() 
{
  // Handle Client
  server.handleClient();

  // Control movement of motor
  g_AccelStepper.run(); 

  // If motor it not running then disable motor pin outputs
  if ( g_AccelStepper.isRunning() == 0)
  {
    // Keep the motor cool
    if (g_data.stepper_motor_and_driver == 0)
    {
      // 28BYJ-48 + ULN: Disable outputs of motor pins
      g_AccelStepper.disableOutputs();
    }
    else
    {
      // Nema17 + A4988: set 'sleep' pin to LOW again
      pinMode( MOTOR_PIN3, OUTPUT);
      digitalWrite( MOTOR_PIN3, LOW );
    }   
  }

  // Check if WiFi is still connected
  if ( 
       ( g_wifi_mode == 1)
       &&
       (WiFi.status() != WL_CONNECTED )
     )
  {
    wifi_reconnect();
  }
}

// -----------------------------------------------------------------------------------------
// wifi_reconnect
// Automatically reconnect to your known WiFi network in case you got disconnected
// -----------------------------------------------------------------------------------------
void wifi_reconnect()
{
  // Control movement of motor
  g_AccelStepper.run(); 

  // Try to disconnect WiFi
  Serial.println("WiFi disconnected");
  WiFi.disconnect(true);

  // Starting WiFi again
  Serial.println("Starting WiFi again");

  Serial.print("Connecting to : '");
  Serial.print(g_data.ssid);
  Serial.println("'");

  WiFi.mode(WIFI_STA);
  WiFi.begin(g_data.ssid, g_data.password);

  int i = 0;
  while (WiFi.status() != WL_CONNECTED && i < 31)
  {
    delay(500);
    Serial.print(".");
    ++i;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
      Serial.println("");
      Serial.print("Connected to  : '");
      Serial.print(g_data.ssid);
      Serial.println("'");
      Serial.print("IP address    : ");
      Serial.println(WiFi.localIP());
  }
}

// -----------------------------------------------------------------------------------------
// move_motor
// Move motor the provided number of rotations
// -----------------------------------------------------------------------------------------
float move_motor (float no_rotations)
{ 
  // If motor is busy with moving, then ignore the request
  if ( g_AccelStepper.isRunning() == 1)
  {
    Serial.println("Motor still running, ignoring request to move");
    return (0.0);
  }
  else
  {   
    Serial.print("Motor Positions: left='");
    Serial.print(g_data.position_left);
    Serial.print("', previous='");
    Serial.print(g_data.position_current_previous);
    Serial.print("', current='");
    Serial.print(g_data.position_current);
    Serial.print("', right='");
    Serial.print(g_data.position_right);
    Serial.println("'");
  
    // Check if you do not go below the 'left' threshold
    if ( (g_data.position_current + no_rotations) < g_data.position_left )
    {
      no_rotations = g_data.position_left - g_data.position_current;
      
      g_data.position_current_previous = g_data.position_current;
      g_data.position_current          = g_data.position_left;
    }
  
    // Check if you do not go above the 'right' threshold
    else if ( (g_data.position_current + no_rotations) > g_data.position_right )
    {
      no_rotations = g_data.position_right - g_data.position_current;

      g_data.position_current_previous = g_data.position_current;
      g_data.position_current          = g_data.position_right;
    }
    // No threshold exceeded
    else
    {
      g_data.position_current_previous = g_data.position_current;
      g_data.position_current          = g_data.position_current + no_rotations;
    }
  
    Serial.print("Motor will move to position '");
    Serial.print(g_data.position_current);
    Serial.print("' (using '");
    Serial.print(no_rotations);
    Serial.println("' rotations)");
  
    // Update expected new current position into EEPROM
    EEPROM.put( g_start_eeprom_address, g_data);
    EEPROM.commit();

    // Flash internal LED
    flash_internal_led();

    // Check which motor driver is used
    if (g_data.stepper_motor_and_driver == 0)
    {
      // 28BYJ-48 + ULN: Enable outputs of motor pins
      g_AccelStepper.enableOutputs();
    }
    else
    {
      // Nema17 + A4988: set 'sleep' pin to HIGH again
      pinMode( MOTOR_PIN3, OUTPUT);
      digitalWrite( MOTOR_PIN3, HIGH );
    }

    // Initialize current position in internal administration of AccelStepper Library
    // This way you can determine the current position when 'stop' is pressed
    g_AccelStepper.setCurrentPosition ( 0 );
    
    // Give stepper instruction to move the motor
    g_AccelStepper.move( no_rotations * g_data.steps_per_revolution);
  }
  return( no_rotations );
}

// -----------------------------------------------------------------------------------------
// handle_api
// API commands you can send to the Webserver in format: action, value, api. 
// (because of security reasons the 'api' is mandatory)
//
// Example: http://ip-address/api?action=set_host&value=my_host
//
// Currently next API actions are supported:
//
// ESP8266 Functions:
// reboot, value              // Reboot in case 'value' = "true"
// reset, value               // Erase EEPROM in case 'value' == "true"
//
// Miscellaneous Functions:
// set_api, value             // Set new API key to 'value'
// set_host, value            // Set hostname to 'value'
// set_language, value        // Set Web-Interface Language (0=English, 1=Dutch)
//
// Initialization Functions:
// reset_left                 // Reset 'left' treshold to default value
// reset_current              // Reset current position
// reset_right                // Reset 'left' treshold to default value
//
// Stepper Motor Functions:
// set_left, [value]          // Set 'left' threshold position for motor 
//                            // (if value is omittted, current position is used)
// move_motor, value          // Move motor with 'value' steps (can be negative). 
//                            // You cannot move below or above tresholds (left/right)
// set_right, [value]         // Set 'right' threshold position for motor 
//                            // (if value is omittted, current position is used)
//
// stop_motor                 // Stop current motor movement
//
// motor_acceleration, value  // Set Motor Acceleration
// motor_max_speed, value     // Set Motor Maximum Speed
//
// move_total_left            // Move to 'left' treshold position
// move_total_right           // Move to 'right' treshold position
// -----------------------------------------------------------------------------------------
void handle_api()
{
  // Get variables for all commands (action,value,api)
  String action = server.arg("action");
  String value = server.arg("value");
  String api = server.arg("api");

  char buffer[256];
  sprintf( buffer, "\nWebserver API called with parameters (action='%s', value='%s', api='%s')\n\n", action.c_str(), value.c_str(), api.c_str() );

  Serial.print( buffer );

  // First check if user wants to set new API key (only possible when not already set)
  if ( (action == "set_api") && (g_data.apikey_set == 0) && (strcmp(api.c_str(), g_default_api_key)) )
  {
    Serial.println("Handle 'set_api' action");

    // Flash internal LED
    flash_internal_led();

    // Make sure that API key is not bigger as 64 characters (= LENGTH_API_KEY)
    if ( strlen(api.c_str()) > LENGTH_API_KEY )
    {
      char buffer[256];
      sprintf( buffer, "NOK (API key '%s' is bigger then '%d' characters)", api.c_str(), LENGTH_API_KEY );
      
      server.send ( 501, "text/html", buffer);
      delay(200);
    }
    else
    {
      Serial.print("Setting API key to value: '");
      Serial.print(value);
      Serial.println("'");
  
      strcpy( g_data.apikey, value.c_str() );
      g_data.apikey_set = 1;
  
      // replace values in EEPROM
      EEPROM.put( g_start_eeprom_address, g_data);
      EEPROM.commit();
  
      char buffer[256];
      sprintf( buffer, "OK (API key set to value '%s')", g_data.apikey );
      
      server.send ( 200, "text/html", buffer);
      delay(200);
    }
  }

  // API key valid?
  if (strcmp (api.c_str(), g_data.apikey) != 0 )
  {
    char buffer[256];
    sprintf( buffer, "NOK (you are not authorized to perform an action without a valid API key, provided api-key '%s')", api.c_str() );
    
    server.send ( 501, "text/html", buffer);
    delay(200);
  }
  else
  {
    // Action: reboot
    if (action == "reboot")
    {
      Serial.println("Handle 'reboot' action");

      server.send ( 200, "text/html", "OK");
      delay(500);

      // Flash internal LED
      flash_internal_led();

      if ( strcmp(value.c_str(), "true") == 0 )
      {
        Serial.println("Rebooting ESP8266");
        ESP.restart();
      }
    }
    // Action: reset
    else if (action == "reset")
    {
      Serial.println("Handle 'reset' action");

      server.send ( 200, "text/html", "OK");
      delay(200);

      if ( strcmp(value.c_str(), "true") == 0 )
      {
        // Flash internal LED
        flash_internal_led();
        
        Serial.println("Clearing EEPROM values of ESP8266");

        // Fill used EEPROM block with 00 bytes
        for (unsigned int i = g_start_eeprom_address; i < (g_start_eeprom_address + ALLOCATED_EEPROM_BLOCK); i++)
        {
          EEPROM.put(i, 0);
        }    
        // Commit changes to EEPROM
        EEPROM.commit();
  
        ESP.restart();
      }
    }
    // Action: set_host
    else if (action == "set_host")
    {
      Serial.println("Handle 'set_host' action");

      // Make sure that value is supplied for hostname
      if ( value == "" )
      {
        Serial.println("No hostname supplied for 'set_host' action");
        
        server.send ( 501, "text/html", "NOK (no hostname supplied for 'set_host' action)");
        delay(200);
      }
      else
      {
        // Flash internal LED
        flash_internal_led();
        
        // Set new Hostname
        strcpy( g_data.hostname, value.c_str() );
  
        // Replace values in EEPROM
        EEPROM.put( g_start_eeprom_address, g_data);
        EEPROM.commit();
  
        Serial.print("Setting hostname to: '");
        Serial.print(value);
        Serial.println("'");
  
        // Set Hostname
        WiFi.hostname(value);         

        char buffer[256];
        sprintf( buffer, "OK (hostname set to value '%s')", value.c_str() );
        
        server.send ( 200, "text/html", buffer);
        delay(200);
      }
    }
    // Action: set_language
    else if (action == "set_language")
    {
      Serial.println("Handle 'set_language' action");

      // Make sure that value is supplied for hostname
      if (value == "")
      {
        Serial.println("No language supplied for 'set_language' action, falling back to English");

        g_data.language = 0;
        
        // Replace values in EEPROM
        EEPROM.put( g_start_eeprom_address, g_data);
        EEPROM.commit();
       
        server.send ( 501, "text/html", "NOK (no language supplied for 'set_language' action, falling back to English");
        delay(200);
      }
      else if ( (value.toInt() < 0) || (value.toInt() > 1) )
      {
        Serial.println("Unknown language supplied for 'set_language' action, falling back to English");

        g_data.language = 0;
        
        // Replace values in EEPROM
        EEPROM.put( g_start_eeprom_address, g_data);
        EEPROM.commit();
       
        server.send ( 501, "text/html", "NOK (unknown language supplied for 'set_language' action, falling back to English");
        delay(200);
      }
      else
      {
        // Flash internal LED
        flash_internal_led();

        g_data.language = value.toInt();
        
        // Replace values in EEPROM
        EEPROM.put( g_start_eeprom_address, g_data);
        EEPROM.commit();

        char buffer [256];
        sprintf( buffer, "OK (language set to value '%d')", g_data.language );

        server.send ( 200, "text/html", buffer);
        delay(200);
      }
    }   
    // Action: reset_left
    else if (action == "reset_left")
    {
      Serial.println("Handle 'reset_left' action");

      // Flash internal LED
      flash_internal_led();

      g_data.position_left = DEFAULT_POSITION_LEFT;

      // Replace values in EEPROM
      EEPROM.put( g_start_eeprom_address, g_data);
      EEPROM.commit();

      server.send ( 200, "text/html", "OK (treshold 'left' set to default value)");
      delay(200);
    }
    // Action: reset_current
    else if (action == "reset_current")
    {
      Serial.println("Handle 'reset_current' action");

      // Flash internal LED
      flash_internal_led();

      g_data.position_current_previous = DEFAULT_POSITION_CURRENT_PREVIOUS;
      g_data.position_current          = DEFAULT_POSITION_CURRENT;

      // Replace values in EEPROM
      EEPROM.put( g_start_eeprom_address, g_data);
      EEPROM.commit();

      server.send ( 200, "text/html", "OK (current position set to default value");
      delay(200);
    }
    // Action: reset_right
    else if (action == "reset_right")
    {
      Serial.println("Handle 'reset_right' action");

      // Flash internal LED
      flash_internal_led();

      g_data.position_right = DEFAULT_POSITION_RIGHT;

      // Replace values in EEPROM
      EEPROM.put( g_start_eeprom_address, g_data);
      EEPROM.commit();

      server.send ( 200, "text/html", "OK (treshold 'right' set to default value)");
      delay(200);
    }
    // Action: set_left
    else if (action == "set_left")
    {
      Serial.println("Handle 'set_left' action");

      // Set new 'left' threshold (if no value is provided then assume current position)
      float position_left = g_data.position_current;
      if (value != "")
      {
        position_left = value.toFloat();
      }

      // Safety check: 'left' threshold cannot be bigger than 'right' treshold
      if ( (position_left > g_data.position_right) || (position_left > g_data.position_current) )
      {
        Serial.println("Treshold 'left' cannot be bigger than 'current' position or treshold 'right' for 'set_motor_left' action");

        Serial.print("Requested: left='");
        Serial.print(position_left);
        Serial.print("', Configured left='");
        Serial.print(g_data.position_left);
        Serial.print("', Current='");
        Serial.print(g_data.position_current);
        Serial.print("', Configured right='");
        Serial.print(g_data.position_right);
        Serial.println("'");
        
        server.send ( 501, "text/html", "NOK (treshold 'left' cannot be bigger than treshold 'right' for 'set_motor_left' action)");
      }
      else
      {
        // Display current 'treshold'
        Serial.print("BEFORE: left=");
        Serial.print(g_data.position_left);
        Serial.print("', cur='");
        Serial.print(g_data.position_current);
        Serial.print("', right='");
        Serial.print(g_data.position_right);
        Serial.println("'");
  
        // Display new 'treshold'
        Serial.print("AFTER : left=");
        Serial.print(position_left);
        Serial.print("', cur='");
        Serial.print(g_data.position_current);
        Serial.print("', right='");
        Serial.print(g_data.position_right);
        Serial.println("'");

        // Flash internal LED
        flash_internal_led();

        // Update new 'left' threshold into EEPROM
        g_data.position_left = position_left;

        EEPROM.put( g_start_eeprom_address, g_data);
        EEPROM.commit();

        char buffer [256];
        sprintf( buffer, "OK (threshold 'left' set to value '%1.2f')", position_left );
  
        server.send ( 200, "text/html", buffer);
        delay(200);      
      }
    }
    // Action: move_motor
    else if (action == "move_motor")
    {
      Serial.println("Handle 'motor' action");

      // Make sure that value is supplied for 'move_motor' action
      if (value == "")
      {
        Serial.println("No value supplied for 'move_motor' action");        

        server.send ( 501, "text/html", "NOK (no value supplied for 'move_motor' action)");        
      }
      else
      {
        // Try to move Motor with provided number of rotations. However
        // depending on the set treshold this can be different as what
        // really will be performed
        Serial.print("Requested to move motor with '");     
        Serial.print(value);
        Serial.println("' rotations");     

        // Flash internal LED
        flash_internal_led();

        float actual_performed_rotations = move_motor( value.toFloat() );      

        char buffer [256];
        sprintf( buffer, "OK (the 'move_motor' action has performed '%1.2f' rotations)", actual_performed_rotations);
        
        server.send ( 200, "text/html", buffer );
        delay(200);
      }
    }
    // Action: set_right
    else if (action == "set_right")
    {
      Serial.println("Handle 'set_right' action");

      // Set new 'right' threshold (if no value is provided then assume current position)
      float position_right = g_data.position_current;
      if (value != "")
      {
        position_right = value.toFloat();
      }

      // Safety check: 'right' threshold cannot be smaller than 'left' treshold
      if ( (position_right < g_data.position_left) || (position_right < g_data.position_current) )
      {
        Serial.println("Treshold 'right' cannot be smaller than 'current' position or treshold 'left'");        

        Serial.print("Requested: right='");
        Serial.print(position_right);
        Serial.print("', Configured left='");
        Serial.print(g_data.position_left);
        Serial.print("', Current='");
        Serial.print(g_data.position_current);
        Serial.print("', Configured right='");
        Serial.print(g_data.position_right);
        Serial.println("'");

        server.send ( 501, "text/html", "NOK (treshold 'right' cannot be smaller than treshold 'left')");
      }
      else
      {
        // Display current 'treshold'
        Serial.print("BEFORE: left=");
        Serial.print(g_data.position_left);
        Serial.print("', cur='");
        Serial.print(g_data.position_current);
        Serial.print("', right='");
        Serial.print(g_data.position_right);
        Serial.println("'");
  
        // Display new 'treshold'
        Serial.print("AFTER : left=");
        Serial.print(g_data.position_left);
        Serial.print("', cur='");
        Serial.print(g_data.position_current);
        Serial.print("', right='");
        Serial.print(position_right);
        Serial.println("'");

        // Flash internal LED
        flash_internal_led();

        // Update new 'right' threshold into EEPROM
        g_data.position_right = position_right;

        EEPROM.put( g_start_eeprom_address, g_data);
        EEPROM.commit();

        char buffer [256];
        sprintf( buffer, "OK (threshold 'right' set to value '%1.2f')", position_right );

        server.send ( 200, "text/html", buffer);
        delay(200);
      }
    }
    // Action: move_total_left
    else if (action == "move_total_left")
    {
      Serial.println("Handle 'move_total_left' action");

      // Only 'move total left' in case 'left' treshold is set
      if ( fabs (g_data.position_left - DEFAULT_POSITION_LEFT) < g_epsilon )
      {
        Serial.println("NOK ('left' treshold not set yet)");
        
        server.send( 501, "text/html", "NOK ('left' treshold not set yet)");
        delay(200);
      }
      else
      {
        // Flash internal LED
        flash_internal_led();

      	// Move motor to the total left
      	move_motor( DEFAULT_POSITION_LEFT );      

        server.send ( 200, "text/html", "OK");
      	delay(200);
      }
    }
    // Action: move_total_right
    else if (action == "move_total_right")
    {
      Serial.println("Handle 'move_total_right' action");

      if ( fabs (g_data.position_right - DEFAULT_POSITION_RIGHT) < g_epsilon )
      {
        Serial.println("NOK ('right' treshold not set yet)");

        server.send( 501, "text/html", "NOK ('right' treshold not set yet)");
        delay(200);
      }
      else
      {
        // Flash internal LED
        flash_internal_led();

        // Move motor to the total right
        move_motor( DEFAULT_POSITION_RIGHT );

        server.send ( 200, "text/html", "OK");
        delay(200);
      }
    }
    // Action: stop_motor
    else if (action == "stop_motor")
    {
      Serial.println("Handle 'stop_motor' action");

      // Flash internal LED
      flash_internal_led();

      // Stop current motor action
      g_AccelStepper.stop();

      // Calculate positions moved
      float positions_moved = (g_AccelStepper.currentPosition() / (g_data.steps_per_revolution * 1.0));

      // Check if motor even moved
      if ( fabs(positions_moved) > g_epsilon )
      {      
        g_data.position_current = g_data.position_current_previous + positions_moved;

        EEPROM.put( g_start_eeprom_address, g_data);
        EEPROM.commit();
      }

      g_AccelStepper.setCurrentPosition ( 0 );

      server.send ( 200, "text/html", "OK");
      delay(200);      
    }
    // Action: motor_acceleration
    else if (action == "motor_acceleration")
    {
      Serial.println("Handle 'motor_acceleration' action");

      // Flash internal LED
      flash_internal_led();

      g_data.motor_acceleration = value.toFloat();
      EEPROM.put( g_start_eeprom_address, g_data);
      EEPROM.commit();

      // Initialize global speed settings for the Motor Driver 

      // Set acceleration
      g_AccelStepper.setAcceleration ( g_data.motor_acceleration );

      // Set maximum motor speed
      g_AccelStepper.setMaxSpeed ( g_data.motor_max_speed );

      server.send ( 200, "text/html", "OK");
      delay(200);      
    }
    else if (action == "motor_max_speed")
    {
      Serial.println("Handle 'motor_max_speed' action");

      // Flash internal LED
      flash_internal_led();

      g_data.motor_max_speed = value.toFloat();

      EEPROM.put( g_start_eeprom_address, g_data);
      EEPROM.commit();

      // Initialize global speed settings for the Motor Driver 

      // Set acceleration
      g_AccelStepper.setAcceleration ( g_data.motor_acceleration );

      // Set maximum motor speed
      g_AccelStepper.setMaxSpeed ( g_data.motor_max_speed );

      server.send ( 200, "text/html", "OK");
      delay(200);      
    }
    // Unknown action supplied :-(
    else
    {
      Serial.println("Unknown action");
      server.send ( 501, "text/html", "NOK (unknown action supplied)");      
    }
  }
}

// -----------------------------------------------------------------------------------------
// flash_internal_led()
// Flash internal LED of ESP8266 board
// -----------------------------------------------------------------------------------------
void flash_internal_led()
{
  // Set internal LED on
  digitalWrite(BUILTIN_LED1, LOW);
  delay(10);

  // Set internal LED off 
  digitalWrite(BUILTIN_LED1, HIGH);
  delay(10);
}

// -----------------------------------------------------------------------------------------
// handle_format
// Format Flash ROM of ESP8266 Flash
// -----------------------------------------------------------------------------------------
void handle_format()
{
  server.send ( 200, "text/html", "OK");
  Serial.println("Format SPIFFS");
  if (SPIFFS.format())
  {
    if (!SPIFFS.begin())
    {
      Serial.println("Format SPIFFS failed");
    }
  }
  else
  {
    Serial.println("Format SPIFFS failed");
  }
  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS failed, needs formatting");
  }
  else
  {
    Serial.println("SPIFFS mounted");
  }
}

// -----------------------------------------------------------------------------------------
// handle_erase
// Erase the EEPROM contents of the ESP8266 board
// -----------------------------------------------------------------------------------------
void handle_erase()
{
  server.send ( 200, "text/html", "OK");
  Serial.println("Erase complete EEPROM");

  for (unsigned int i = 0 ; i < ALLOCATED_EEPROM_BLOCK; i++) 
  {
    EEPROM.write(i, 0);
  }  
  // Commit changes to EEPROM
  EEPROM.commit();

  // Restart board
  ESP.restart();
}

// -----------------------------------------------------------------------------------------
// is_authenticated(int set_cookie)
// Helper-method for authenticating for Web Access
// -----------------------------------------------------------------------------------------
bool is_authenticated(int set_cookie)
{
  MD5Builder md5;

  // Create unique MD5 value (hash) of Web Password
  md5.begin();
  md5.add(g_data.web_password);
  md5.calculate();

  String web_pass_md5 = md5.toString();
  String valid_cookie = "eCurtains=" + web_pass_md5;

  // Check if web-page has Cookie data
  if (server.hasHeader("Cookie"))
  {
    // Check if Cookie of web-session corresponds with our Cookie of eCurtains
    String cookie = server.header("Cookie");

    if (cookie.indexOf(valid_cookie) != -1)
    {
      // No, then set our Cookie for this Web session
      if (set_cookie == 1)
      {
        server.sendHeader("Set-Cookie","eCurtains=" + web_pass_md5 + "; max-age=86400");
      }
      return true;
    }
  }
  return false;
}

// -----------------------------------------------------------------------------------------
// handle_login_html()
// -----------------------------------------------------------------------------------------
void handle_login_html()
{
  String login_html_page;

  login_html_page  = "<title>" + l_login_screen[g_data.language] + "</title>";
  login_html_page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>";

  login_html_page += "</head><body><h1>" + l_login_screen[g_data.language] + "</h1>";

  login_html_page += "<html><body>";
  login_html_page += "<form action='/login_ajax' method='POST'>" + l_provide_credentials[g_data.language];
  login_html_page += "<br>";
  login_html_page += "<input type='hidden' name='form' value='login'>";
  login_html_page += "<pre>";

  char length_web_user[10];
  sprintf( length_web_user, "%d", LENGTH_WEB_USER );
  String str_length_web_user = length_web_user;

  char length_web_password[10];
  sprintf( length_web_password, "%d", LENGTH_WEB_PASSWORD );
  String str_length_web_password = length_web_password;
 
  login_html_page += l_user_label[g_data.language] + "<input type='text' name='user' placeholder='" + l_username[g_data.language] + "' maxlength='" + str_length_web_user + "'>";
  login_html_page += "<br>";
  login_html_page += l_password_label[g_data.language] + "<input type='password' name='password' placeholder='" + l_password[g_data.language] + "' maxlength='" + str_length_web_password + "'>";
  login_html_page += "</pre>";
  login_html_page += "<input type='submit' name='Submit' value='Submit'></form>";
  login_html_page += "<br><br>";
  login_html_page += "</body></html>";
  
  server.send(200, "text/html", login_html_page);
}

// -----------------------------------------------------------------------------------------
// handle_login_ajax()
// -----------------------------------------------------------------------------------------
void handle_login_ajax()
{
  String form   = server.arg("form");
  String action = server.arg("action");
  
  if (form == "login")
  {
    String user_arg = server.arg("user");
    String pass_arg = server.arg("password");

    // Calculate MD5 (hash) of set web-password in EEPROM
    MD5Builder md5;   
    md5.begin();
    md5.add(g_data.web_password);
    md5.calculate();
    String web_pass_md5 = md5.toString();

    // Check if Credentials are OK and if yes set the Cookie for this session
    if (user_arg == g_data.web_user && pass_arg == g_data.web_password)
    {
      String header = "HTTP/1.1 301 OK\r\nSet-Cookie: eCurtains=" + web_pass_md5 + "; max-age=86400\r\nLocation: /\r\nCache-Control: no-cache\r\n\r\n";
      server.sendContent(header);
    }
    else
    {
      Serial.println("Wrong user-name and/or password supplied");

      // Reset Cookie and present login screen again
      String header = "HTTP/1.1 301 OK\r\nSet-Cookie: eCurtains=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
      server.sendContent(header);
    }
  }
  else
  {
    server.send ( 200, "text/html", "Nothing");
  }
}

// -----------------------------------------------------------------------------------------
// handle_settings_html()
// -----------------------------------------------------------------------------------------
void handle_settings_html()
{
  // Check if you are authorized to see the Settings dialog
  if ( !is_authenticated(0) )
  {
    Serial.println("Unauthorized to change Settings");

    // Reset Cookie and present login screen
    String header = "HTTP/1.1 301 OK\r\nSet-Cookie: eCurtains=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    server.sendContent(header);
    
    server.send ( 200, "text/html", "Unauthorized");
  }
  else
  { 
    String settings_html_page;
  
    settings_html_page  = "<title>" + l_settings_screen[g_data.language] + "</title>";
    settings_html_page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>";
  
    settings_html_page += "</head><body><h1>" + l_settings_screen[g_data.language] + "</h1>";
  
    settings_html_page += "<html><body>";
    settings_html_page += "<form action='/settings_ajax' method='POST'>";
    settings_html_page += l_provide_new_credentials[g_data.language];
    settings_html_page += "<br>";
    settings_html_page += "<input type='hidden' name='form' value='settings'>";
    settings_html_page += "<pre>";

    char length_web_user[10];
    sprintf( length_web_user, "%d", LENGTH_WEB_USER );
    String str_length_web_user = length_web_user;

    char length_web_password[10];
    sprintf( length_web_password, "%d", LENGTH_WEB_PASSWORD );
    String str_length_web_password = length_web_password;

    char length_api_key[10];
    sprintf( length_api_key, "%d", LENGTH_API_KEY );
    String str_length_api_key = length_api_key;
    
    settings_html_page += l_new_user_label[g_data.language] + "<input type='text' name='user' value='";
    settings_html_page += g_data.web_user;
    settings_html_page += "' maxlength='" + str_length_web_user + "'>";
    settings_html_page += "<br>";
    settings_html_page += l_new_password_label[g_data.language] + "<input type='text' name='password' value='";
    settings_html_page += g_data.web_password;
    settings_html_page += "' maxlength='" + str_length_web_password + "'>";
    settings_html_page += "<br>";
    settings_html_page += l_new_api_key[g_data.language] + "<input type='text' name='apikey' value='";
    settings_html_page += g_data.apikey;
    settings_html_page += "' maxlength='" + str_length_api_key + "'>";
    settings_html_page += "</pre>";
    settings_html_page += "<br>";

    settings_html_page += l_change_language_hostname[g_data.language];
    settings_html_page += "<br>";   
    settings_html_page += "<pre>";
    settings_html_page += l_language_label[g_data.language] + "<select name='language'>";

    // Default English
    if (g_data.language == 0)
    {
      settings_html_page += "<option selected>";
      settings_html_page += l_language_english[g_data.language];
      settings_html_page += "</option><option>";
      settings_html_page += l_language_dutch[g_data.language];
      settings_html_page += "</option></select>";
    }
    // Otherwise Dutch
    else
    {
      settings_html_page += "<option>";
      settings_html_page += l_language_english[g_data.language];
      settings_html_page += "</option><option selected>";
      settings_html_page += l_language_dutch[g_data.language];
      settings_html_page += "</option></select>";
    }

    char length_hostname[10];
    sprintf( length_hostname, "%d", LENGTH_HOSTNAME );
    String str_length_hostname = length_hostname;

    settings_html_page += "<br>";
    settings_html_page += l_hostname_label[g_data.language] + "<input type='text' name='hostname' value='";
    settings_html_page += g_data.hostname;
    settings_html_page += "' maxlength='" + str_length_hostname + "'>"; 
    settings_html_page += "</pre>";
    settings_html_page += "<br>";

    settings_html_page += l_change_motor_settings[g_data.language];
    settings_html_page += "<br>";   
    settings_html_page += "<pre>";

    settings_html_page += l_change_step_motor_and_driver[g_data.language] + "<select name='motor_and_driver'>";

    // Default 28BYJ-48 + ULN
    if (g_data.stepper_motor_and_driver == 0)
    {
      settings_html_page += "<option selected>";
      settings_html_page += l_28BYJ_ULN[g_data.language];
      settings_html_page += "</option><option>";
      settings_html_page += l_Nema17_A4988[g_data.language];
      settings_html_page += "</option></select>";
    }
    // Otherwise NEMA17 + A4988
    else
    {
      settings_html_page += "<option>";
      settings_html_page += l_28BYJ_ULN[g_data.language];
      settings_html_page += "</option><option selected>";
      settings_html_page += l_Nema17_A4988[g_data.language];
      settings_html_page += "</option></select>";
    }

    settings_html_page += "<br>";
    
    settings_html_page += l_change_steps_per_revolution[g_data.language] + "<input type='text' name='stepsperrevolution' value='";
    settings_html_page += g_data.steps_per_revolution;
    settings_html_page += "'>"; 
    settings_html_page += "<br>";  
    settings_html_page += l_change_motor_acceleration[g_data.language] + "<input type='text' name='acceleration' value='";
    settings_html_page += g_data.motor_acceleration;
    settings_html_page += "'>"; 
    settings_html_page += "<br>";
    settings_html_page += l_change_motor_max_speed[g_data.language] + "<input type='text' name='max_speed' value='";
    settings_html_page += g_data.motor_max_speed;
    settings_html_page += "'>"; 
    settings_html_page += "</pre>";
    settings_html_page += "<br>";

    settings_html_page += l_login_again[g_data.language];
    settings_html_page += "<br><br>";
     
    settings_html_page += "<input type='submit' name='Submit' value='Submit'></form>";
    settings_html_page += "<br><br>";
    settings_html_page += "</body></html>";
    
    server.send(200, "text/html", settings_html_page);
  }
}
  
// -----------------------------------------------------------------------------------------
// handle_settings_ajax()
// -----------------------------------------------------------------------------------------
void handle_settings_ajax()
{
  String form   = server.arg("form");
  String action = server.arg("action");
  
  if (form == "settings")
  {
    String user_arg  = server.arg("user");
    String pass_arg  = server.arg("password");
    String api_arg   = server.arg("apikey");
    String host_arg  = server.arg("hostname");
    String lang_arg  = server.arg("language");
    String type_arg  = server.arg("motor_and_driver");
    String rev_arg   = server.arg("stepsperrevolution");
    String acc_arg   = server.arg("acceleration");
    String speed_arg = server.arg("max_speed");

    // Check if new web-user of web-password has been set
    int login_again = 0;
    if ( (strcmp (g_data.web_user, user_arg.c_str()) != 0) || (strcmp(g_data.web_password, pass_arg.c_str()) != 0) )
    {
      login_again = 1;
    }
    
    // Set new Web User and Web Password
    strcpy( g_data.web_user,     user_arg.c_str() );
    strcpy( g_data.web_password, pass_arg.c_str() );
    strcpy( g_data.hostname,     host_arg.c_str() );

    // Set new API key
    strcpy( g_data.apikey,       api_arg.c_str() );
    
    // Set new language
    if ( strcmp( lang_arg.c_str(), l_language_english[g_data.language].c_str()) == 0 )
    {
      g_data.language = 0;
    }
    else
    {
      g_data.language = 1;
    }

    // Set new 'Stepper Motor & Driver', 'Steps Per Revolution, 'Motor Acceleration' and 'Max Speed'
    if ( strcmp( type_arg.c_str(), l_28BYJ_ULN[g_data.language].c_str()) == 0 )
    {
      g_data.stepper_motor_and_driver = 0;
      g_data.steps_per_revolution = STEPS_PER_REVOLUTION_28BYJ;
    }
    else
    {
      g_data.stepper_motor_and_driver = 1;
      g_data.steps_per_revolution = STEPS_PER_REVOLUTION_NEMA17;
    }
    
    g_data.motor_acceleration   = acc_arg.toFloat();
    g_data.motor_max_speed      = speed_arg.toFloat();
    
    // Set Hostname
    WiFi.hostname(g_data.hostname);         

    // Initialize AccelStepper library
    if (g_data.stepper_motor_and_driver == 0)
    {  
      // 28BYJ-48 + ULN
      // Remark: use pin sequence: IN1-IN3-IN2-IN4 when using the AccelStepper with 28BYJ-48      
      g_AccelStepper = AccelStepper( AccelStepper::FULL4WIRE, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4 );

      Serial.println("Initializing AccelStepper for 28BYJ-48 + ULN");
      Serial.println();
    }
    else
    {
      // Nema17 + A4988
      // Remark: connect MOTOR_PIN1 to 'direction' and MOTOR_PIN2 to 'steps' of A4988 Driver
      g_AccelStepper = AccelStepper( AccelStepper::DRIVER, MOTOR_PIN2, MOTOR_PIN1 );

      // Set 'reset' pin to HIGH
      pinMode( MOTOR_PIN4, OUTPUT);
      digitalWrite( MOTOR_PIN4, HIGH );

      Serial.println("Initializing AccelStepper for Nema17 + A4988");
      Serial.println();
    }
    
    // Set acceleration
    g_AccelStepper.setAcceleration ( g_data.motor_acceleration );
  
    // Set maximum motor speed
    g_AccelStepper.setMaxSpeed ( g_data.motor_max_speed );

    // Update values in EEPROM
    EEPROM.put( g_start_eeprom_address, g_data);
    EEPROM.commit();

    char buffer[256];
    sprintf(buffer,"New webuser-name='%s', new webuser-password='%s'", g_data.web_user, g_data.web_password);
    Serial.println(buffer);

    // Force login again in case web-user or web-password is changed
    if (login_again == 1)
    {
      // Reset Cookie and present login screen again
      String header = "HTTP/1.1 301 OK\r\nSet-Cookie: eCurtains=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
      server.sendContent(header);
    }
    // Otherwise present main Web-Interface again
    else
    {
      String header = "HTTP/1.1 301 OK\r\nLocation: /\r\nCache-Control: no-cache\r\n\r\n";
      server.sendContent(header);      
    }
  }
}

// -----------------------------------------------------------------------------------------
// get_page
// Display eCurtains Web-Interface
// -----------------------------------------------------------------------------------------
String get_page()
{
  String page;
  
  page  = "<html lang=nl-NL><head><meta http-equiv='refresh' content='5'/>";  

  page += "<title>eCurtains</title>";
  page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>";
  page += "</head><body><h1>eCurtains Web-Interface</h1>";

  page += l_version[g_data.language] + "'";
  page += g_data.version;
  page += "', ";
  
  page += l_hostname[g_data.language] + "'";
  page += g_data.hostname;
  page += "'";
  page += "<br>";
  page += "<br>";

  page += "<form action='/' method='POST'>";
  page += "<input type='submit' name='logout' value='" + l_btn_logout[g_data.language] + "'>";
  page += "<input type='submit' name='settings' value='" + l_btn_settings[g_data.language] + "'> ";

  page += "<h3>" + l_overview_header[g_data.language] + "</h3>";
  page += "<pre>";
  page += l_overview_1[g_data.language] + "<br>";
  page += l_overview_2[g_data.language] + "<br>";
  page += l_overview_3[g_data.language] + "<br>";
  page += l_overview_4[g_data.language] + "<br>";
  page += "</pre>";
  page += l_overview_5[g_data.language];

  page += "<h3>" + l_positions[g_data.language] + "</h3>";
  page += "<ul><li>" + l_treshold_left[g_data.language];
  page += g_data.position_left;
  page += "</li>";
  page += "<li>" + l_previous_position[g_data.language];
  page += g_data.position_current_previous;
  page += "</li>";
  page += "<li>" + l_current_position[g_data.language];
  page += g_data.position_current;
  page += "</li>";
  page += "<li>" + l_treshold_right[g_data.language];
  page += g_data.position_right;
  page += "</li></ul>";
  
  page += "<h3>Reset</h3>";
  page += "<input type='submit' name='reset_left' value='" + l_btn_treshold_left[g_data.language] + "'>";
  page += "<input type='submit' name='reset_current' value='" + l_btn_current_position[g_data.language] + "'>";
  page += "<input type='submit' name='reset_right' value='" + l_btn_treshold_right[g_data.language] + "'>";

  page += "<h3>" + l_move_motor[g_data.language] + "</h3>"; 
  page += "<input type='submit' name='left100' value='" + l_btn_left_100[g_data.language] + "'>";
  page += "<input type='submit' name='left10' value='" + l_btn_left_10[g_data.language] + "'>";
  page += "<input type='submit' name='left1' value='" + l_btn_left_1[g_data.language] + "'>";
  page += "<input type='submit' name='left0.5' value='" + l_btn_left_0_5[g_data.language] + "'>";
  page += "<input type='submit' name='left0.25' value='" + l_btn_left_0_25[g_data.language] + "'>";
  page += "<input type='submit' name='left0.1' value='" + l_btn_left_0_1[g_data.language] + "'>";

  page += "<br><br>";
  page += "<input type='submit' name='stop_motor' value='" + l_stop_motor[g_data.language] + "'>";
  page += "<br><br>";
 
  page += "<input type='submit' name='right0.1' value='" + l_btn_right_0_1[g_data.language] + "'>";
  page += "<input type='submit' name='right0.25' value='" + l_btn_right_0_25[g_data.language] + "'>";
  page += "<input type='submit' name='right0.5' value='" + l_btn_right_0_5[g_data.language] + "'>";
  page += "<input type='submit' name='right1' value='" + l_btn_right_1[g_data.language] + "'>";
  page += "<input type='submit' name='right10' value='" + l_btn_right_10[g_data.language] + "'>";
  page += "<input type='submit' name='right100' value='" + l_btn_right_100[g_data.language] + "'>";

  page += "<h3>" + l_set_new_tresholds[g_data.language] + "</h3>";
  page += "<input type='submit' name='set_left' value='" + l_btn_treshold_left[g_data.language] + "'>";
  page += "<input type='submit' name='set_right' value=' " + l_btn_treshold_right[g_data.language] + "'>";
  
  page += "<h3>" + l_move_to_tresholds[g_data.language] + "</h3>";
  page += "<input type='submit' name='move_total_left' value='" + l_btn_move_total_left[g_data.language] + "'>";
  page += "<input type='submit' name='move_total_right' value='" + l_btn_move_total_right[g_data.language] + "'>";

  page += "</body></html>";
  
  return (page);
}

// -----------------------------------------------------------------------------------------
// handle_upgradefw_html
// Handle upgrading of firmware of ESP8266 device
// -----------------------------------------------------------------------------------------
void handle_upgradefw_html()
{
  // Check if you are authorized to upgrade the firmware
  if ( !is_authenticated(0) )
  {
    Serial.println("Unauthorized to upgrade firmware");

    // Reset Cookie and present login screen
    String header = "HTTP/1.1 301 OK\r\nSet-Cookie: eCurtains=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    server.sendContent(header);
    
    server.send ( 200, "text/html", "Unauthorized");
  }
  else
  {
    String upgradefw_html_page;
  
    upgradefw_html_page  = "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>";
    upgradefw_html_page += "</head><body><h1>Upgrade firmware</h1>";
    upgradefw_html_page += "<br>";
    upgradefw_html_page += "<form method='POST' action='/upgradefw2' enctype='multipart/form-data'>";
    upgradefw_html_page += "<input type='file' name='upgrade'>";
    upgradefw_html_page += "<input type='submit' value='Upgrade'>";
    upgradefw_html_page += "</form>";
    upgradefw_html_page += "<b>" + l_after_upgrade[g_data.language] + "</b>";
  
    server.send ( 200, "text/html", upgradefw_html_page);
  }
}

// -----------------------------------------------------------------------------------------
// handle_wifi_html
// Simple WiFi setup dialog
// -----------------------------------------------------------------------------------------
void handle_wifi_html()
{
  String wifi_page;

  wifi_page  = "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>";
  wifi_page += "</head><body><h1>" + l_configure_wifi_network[g_data.language] + "</h1>";
  wifi_page += "<html><body>";
  wifi_page += "<form action='/wifi_ajax' method='POST'>" + l_enter_wifi_credentials[g_data.language];
  wifi_page += "<br>";  
  wifi_page += "<input type='hidden' name='form' value='wifi'>";  

  char length_ssid[10];
  sprintf( length_ssid, "%d", LENGTH_SSID );
  String str_length_ssid = length_ssid;

  char length_ssid_password[10];
  sprintf( length_ssid_password, "%d", LENGTH_PASSWORD );
  String str_length_ssid_password = length_ssid_password;
  
  wifi_page += "<pre>";
  wifi_page += l_network_ssid_label[g_data.language] + "<input type='text' name='ssid' placeholder='" + l_network_ssid_hint[g_data.language] + "' maxlength='" + str_length_ssid + "'>";
  wifi_page += "<br>";
  wifi_page += l_network_password_label[g_data.language] + "<input type='password' name='password' placeholder='" + l_network_password_hint[g_data.language] + "' maxlength='" + str_length_ssid_password + "'>";
  wifi_page += "</pre>";
  wifi_page += "<input type='submit' name='Submit' value='Submit'></form>";
  wifi_page += "<br><br>";
  wifi_page += "</body></html>";

  server.send ( 200, "text/html", wifi_page);
}

// -----------------------------------------------------------------------------------------
// handle_wifi_ajax
// Handle simple WiFi setup dialog configuration
// -----------------------------------------------------------------------------------------
void handle_wifi_ajax()
{
  String form = server.arg("form");

  if (form == "wifi")
  {
    String ssidArg = server.arg("ssid");
    String passArg = server.arg("password");

    strcpy( g_data.ssid, ssidArg.c_str() );
    strcpy( g_data.password, passArg.c_str() );

    // Update values in EEPROM
    EEPROM.put( g_start_eeprom_address, g_data);
    EEPROM.commit();
  
    server.send ( 200, "text/html", "OK");
    delay(500);
  
    ESP.restart();
  }
}

// -----------------------------------------------------------------------------------------
// handle_root
// Handle user-interface actions executed on the Web-Interface
// -----------------------------------------------------------------------------------------
void handle_root()
{
  // Check if you are authorized to see the web-interface
  if ( !is_authenticated(0) )
  {
    Serial.println("Unauthorized to access Web-Interface");

    // Reset Cookie and present login screen
    String header = "HTTP/1.1 301 OK\r\nSet-Cookie: eCurtains=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
    server.sendContent(header);
    
    server.send ( 200, "text/html", "Unauthorized");
  }
  else
  {
    // Logout
    if (server.hasArg("logout") )
    {
      handle_logout();
    }
    // Change Credentials
    else if (server.hasArg("settings") )
    {
      handle_settings();
    }
    // Reset Treshold 'left'
    if ( server.hasArg("reset_left") )
    {
      handle_reset_left();
    }
    // Reset current Position
    else if ( server.hasArg("reset_current") )
    {
      handle_reset_current();
    }
    // Reset Treshold 'right'
    else if ( server.hasArg("reset_right") )
    {
      handle_reset_right();
    }
    // Move left pressed with step of 0.1
    else if ( server.hasArg("left0.1") )
    {
      handle_move_left(-0.1);
    }
    // Move left pressed with step of 0.25
    else if ( server.hasArg("left0.25") )
    {
      handle_move_left(-0.25);
    }
    // Move left pressed with step of 0.5
    else if ( server.hasArg("left0.5") )
    {
      handle_move_left(-0.5);
    }
    // Move left pressed with step of 1
    else if ( server.hasArg("left1") )
    {
      handle_move_left(-1.0);
    }
    // Move left pressed with step of 10
    else if ( server.hasArg("left10") )
    {
      handle_move_left(-10.0);
    }
    // Move left pressed with step of 100
    if ( server.hasArg("left100") )
    {
      handle_move_left(-100.0);
    }
    // Stop motor
    else if ( server.hasArg("stop_motor") )
    {
      handle_stop_motor();
    }
    // Move right pressed with step of 0.1
    else if ( server.hasArg("right0.1") )
    {
      handle_move_right(0.1);
    }
    // Move right pressed with step of 0.25
    else if ( server.hasArg("right0.25") )
    {
      handle_move_right(0.25);
    }
    // Move right pressed with step of 0.5
    else if ( server.hasArg("right0.5") )
    {
      handle_move_right(0.5);
    }
    // Move right pressed with step of 1
    else if ( server.hasArg("right1") )
    {
      handle_move_right(1.0);
    }
    // Move right pressed with step of 10
    else if ( server.hasArg("right10") )
    {
      handle_move_right(10.0);
    }
    // Move right pressed with step of 100
    if ( server.hasArg("right100") )
    {
      handle_move_right(100.0);
    }
    // Set 'left' treshold pressed
    else if ( server.hasArg("set_left") )
    {
      handle_set_left();
    }
    // Set 'right' treshold pressed
    else if ( server.hasArg("set_right") )
    {
      handle_set_right();
    }
    // Move to total left
    else if ( server.hasArg("move_total_left") )
    {
      handle_move_total_left();
    }
    // Move to total right
    else if ( server.hasArg("move_total_right") )
    {
      handle_move_total_right();
    }
    // Stop Motor
    else if ( server.hasArg("stop_motor") )
    {
      handle_stop_motor();
    }
    else
    {
      server.send ( 200, "text/html", get_page() );
    }
  }
}

// -----------------------------------------------------------------------------------------
// handle_logout
// Logout from Web-Server
// -----------------------------------------------------------------------------------------
void handle_logout()
{
  Serial.println("[Logout] pressed");

  // Reset Cookie and present login screen
  String header = "HTTP/1.1 301 OK\r\nSet-Cookie: eCurtains=0\r\nLocation: /login\r\nCache-Control: no-cache\r\n\r\n";
  server.sendContent(header);
}

// -----------------------------------------------------------------------------------------
// handle_settings
// Change settings for Web-Server
// -----------------------------------------------------------------------------------------
void handle_settings()
{
  Serial.println("[Settings] pressed");

  // Present the 'Settings' screen
  String header = "HTTP/1.1 301 OK\r\nLocation: /settings\r\nCache-Control: no-cache\r\n\r\n";
  server.sendContent(header);
}

// -----------------------------------------------------------------------------------------
// handle_reset_left
// Reset treshold 'left' to default value
// -----------------------------------------------------------------------------------------
void handle_reset_left()
{
  Serial.println("[Reset Left] pressed");

  // Flash internal LED
  flash_internal_led();
  
  g_data.position_left = DEFAULT_POSITION_LEFT;

  EEPROM.put( g_start_eeprom_address, g_data);
  EEPROM.commit();

  server.send ( 200, "text/html", get_page() );
}

// -----------------------------------------------------------------------------------------
// handle_reset_current
// Reset current position to default value
// -----------------------------------------------------------------------------------------
void handle_reset_current()
{
  Serial.println("[Reset Current] pressed");

  // Flash internal LED
  flash_internal_led();

  g_data.position_current_previous = DEFAULT_POSITION_CURRENT_PREVIOUS;
  g_data.position_current          = DEFAULT_POSITION_CURRENT;

  EEPROM.put( g_start_eeprom_address, g_data);
  EEPROM.commit();

  server.send ( 200, "text/html", get_page() );
}

// -----------------------------------------------------------------------------------------
// handle_reset_right
// Reset treshold 'right' to default value
// -----------------------------------------------------------------------------------------
void handle_reset_right()
{
  Serial.println("[Reset Right] pressed");

  // Flash internal LED
  flash_internal_led();

  g_data.position_right = DEFAULT_POSITION_RIGHT;

  EEPROM.put( g_start_eeprom_address, g_data);
  EEPROM.commit();

  server.send ( 200, "text/html", get_page() );
}

// -----------------------------------------------------------------------------------------
// handle_move_left
// Move stepper motor to the left
// -----------------------------------------------------------------------------------------
void handle_move_left(float rotations)
{
  Serial.print("[Move Left] pressed for '");
  Serial.print(rotations);
  Serial.println("' rotations");

  // Move motor to the left with given number of rotations
  move_motor( rotations );      
  
  server.send ( 200, "text/html", get_page() );  
}

// -----------------------------------------------------------------------------------------
// handle_move_right
// Move stepper motor to the right
// -----------------------------------------------------------------------------------------
void handle_move_right(float rotations)
{
  Serial.print("[Move Right] pressed for '");
  Serial.print(rotations);
  Serial.println("' rotations");

  // Move motor to the right with given number of rotations
  move_motor( rotations );      

  server.send ( 200, "text/html", get_page() );  
}

// -----------------------------------------------------------------------------------------
// handle_set_left
// Set 'left' treshold to current position
// -----------------------------------------------------------------------------------------
void handle_set_left()
{
  Serial.println("[Set Left] pressed");

  // Flash internal LED
  flash_internal_led();

  // Update new 'left' threshold into EEPROM
  g_data.position_left = g_data.position_current;

  EEPROM.put( g_start_eeprom_address, g_data);
  EEPROM.commit();

  server.send ( 200, "text/html", get_page());
}

// -----------------------------------------------------------------------------------------
// handle_set_right
// Set 'right' treshold to current position
// -----------------------------------------------------------------------------------------
void handle_set_right()
{
  Serial.println("[Set Right] pressed");

  // Flash internal LED
  flash_internal_led();

  // Update new 'right' threshold into EEPROM
  g_data.position_right = g_data.position_current;

  EEPROM.put( g_start_eeprom_address, g_data);
  EEPROM.commit();

  server.send ( 200, "text/html", get_page() );  
}

// -----------------------------------------------------------------------------------------
// handle_move_total_left
// Move motor to 'left' treshold value
// -----------------------------------------------------------------------------------------
void handle_move_total_left()
{
  Serial.println("[Move Total Left] pressed");

  // First check if 'left' treshold is set
  if ( fabs(g_data.position_left - DEFAULT_POSITION_LEFT) < g_epsilon )
  {
    Serial.println("'left' treshold not set, thus not moving motor");
   
    server.send ( 501, "text/html", get_page());
  }
  else
  {
    // Move motor to the total left
    move_motor( DEFAULT_POSITION_LEFT );      

    server.send ( 200, "text/html", get_page());
  }
}

// -----------------------------------------------------------------------------------------
// handle_move_total_right
// Move motor to 'right' treshold value
// -----------------------------------------------------------------------------------------
void handle_move_total_right()
{
  Serial.println("[Move Total Right] pressed");

  // First check if 'right' treshold is set
  if ( fabs(g_data.position_right - DEFAULT_POSITION_RIGHT) < g_epsilon )
  {
    Serial.println("'right' treshold not set, thus not moving motor");

    server.send ( 501, "text/html", get_page());
  }
  else
  {
    // Move motor to the total right
    move_motor( DEFAULT_POSITION_RIGHT );      

    server.send ( 200, "text/html", get_page());
  }
}

// -----------------------------------------------------------------------------------------
// handle_stop_motor
// Stop motor movement
// -----------------------------------------------------------------------------------------
void handle_stop_motor()
{
  Serial.println("[Stop Motor] pressed");

  // Flash internal LED
  flash_internal_led();

  // Stop any possible motor movement
  g_AccelStepper.stop();

  // Calculate positions moved
  float positions_moved = (g_AccelStepper.currentPosition() / (g_data.steps_per_revolution * 1.0));
  
  Serial.print("Positions moved until stopped = '");
  Serial.print(positions_moved);
  Serial.println("'");

  // Check if motor even moved
  if ( fabs(positions_moved) > g_epsilon )
  {
    g_data.position_current = g_data.position_current_previous + positions_moved;

    EEPROM.put( g_start_eeprom_address, g_data);
    EEPROM.commit();
  }

  g_AccelStepper.setCurrentPosition ( 0 );

  server.send ( 200, "text/html", get_page());
}

// -----------------------------------------------------------------------------------------
// History of 'eCurtains' software
//
// 31-Mar-2018  V1.00   Initial version
// 04-Jun-2018  V1.01   Made Steps per Revolution Configurable 
// 05-Jun-2018  V1.02   Made Stepper Motor and Driver configurable. You can now choose:
//                      - Stepper Motor 28BYJ-48 with ULN Stepper Motor Driver (default)
//                      - Stepper Motor Nema17 with A4988 Stepper Motor Driver
//                      Steps per Revolution will be set implicit accordingly
// 06-Jun=2018  V1.03   Added automatic WiFi reconnect in case WiFi connection drops
// 07-Jul-2018  V1.04   Improved handling of A4988 Stepper Motor Driver. Now driver will
//                      be set off in case motor is not moving (thus reducing temperature
//                      of Motor)
// 14-Aug-2018  V1.05   Improved API key handling
// 15-Aug-2018  V1.06   Improved handling of input fields and EEPROM storage
// 26-Oct-2021  V1.07   Initialize stepper library again when using API to change
//                      acceleration or max motor speed
// -----------------------------------------------------------------------------------------
