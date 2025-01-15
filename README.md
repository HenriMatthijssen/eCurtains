# eCurtains
Software for controlling your curtains with a stepper motor using Arduino ESP8266, '28BYJ-48 Stepper motor and ULN Stepper Motor Driver' or 'Nema17 Stepper Motor and A4988 Stepper Motor Driver'.

For faster stepper motor performance I recommend to use the Nema17 Stepper Motor.

See 'Blog Henri Matthijssen' for instructions for the hardware and setup
http://matthijsseninfo.nl

(c) 2018 Henri Matthijssen (henri@matthijsseninfo.nl)



Please do not distribute without permission of the original author and 
respect his time and work spent on this.


First time you installed the software the ESP8266 will startup in Access-Point (AP) mode.
Connect with you WiFi to this AP using next information:

AP ssid           : eCurtains
AP password       : eCurtains

Enter IP-address 192.168.4.1 in your web-browser. This will present you with dialog to fill
the credentials of your home WiFi Network. Fill your home SSID and password and press the
submit button. Your ESP8266 will reboot and connect automatically to your home WiFi Network.
Now find the assigned IP-address to the ESP8266 and use that again in your web-browser.

Before you can use the Web-Interface you have to login with a valid user-name
and password on location: http://ip-address/login (cookie is valid for 24 hours).

Default user-name : admin
Default password  : notdodo

In the Web-Interface you can change the default user-name / password (among other
settings like language, host-name, step motor type & driver, motor acceleration speed 
and motor maximum speed).

First action you have to do in the software is calibrating your curtains by setting
the 'left' (open) and 'right' (closed) position of your curtains. Either use the
Web-Interface or the API for this.

After calibration you can:

Totally open your curtains by:

- pressing [Move Total Left] button in the Web-Interface
  or
- calling API with: http://ip-address/api?action=move_total_left&api=your_api

Totally close your curtains by:

- pressing [Move Total Right] button in the Web-Interface
  or
- calling API with: http://ip-address/api?action=move_total_right&api=your_api

When using the API you always need to supply a valid value for the API key
(default value=27031969). The API has next format in the URL:

http://ip-address/api?action=xxx&value=yyy&api=zzz

Currently next API actions are supported:

reboot, value              (value=false or true)
reset, value               (value=false or true)

set_api, value             (value=new API key, can only be changed once via API)
set_host, value            (value=new host-name)
set_language, value        (0=English, 1=Dutch (default))

reset_left
reset_current
reset_right

set_left, [value]          (if no value supplied then current position is used)
move_motor, value
set_right, [value]         (if no value supplied then current position is used)

stop_motor

motor_acceleration, value  (value=acceleration)
motor_max_speed, value     (value=maximum speed)

move_total_left
move_total_right

You can upgrade the firmware with the (hidden) firmware upgrade URL:
http://ip-address/upgradefw
(you first needed to be logged in for the above URL to work)

You can erase all settings by clearing the EEPROM with next URL:
http://ip-address/erase
(you first needed to be logged in for the above URL to work)

