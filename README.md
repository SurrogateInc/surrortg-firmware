# SurroRTG Arduino library

This library provides tools to control WiFi microcontrollers through [Surrogate.tv](https://www.surrogate.tv/).
Supported microcontrollers include ESP32 and ESP8266 based boards, as well
as Arduino MKR1000 and Arduino MKR1010. The library enables OTA-updates
automatically, so the code can be uploaded wirelessly to the microcontrollers
after the initial upload.

For usage and installation instructions, see [Surrogate docs](https://docs.surrogate.tv/tcp_bot_games.html).

## Adding wifi and OTA credentials for examples

For more in-depth instructions, see [Surrogate docs](https://docs.surrogate.tv/tcp_bot_games.html).

Before the examples can be compiled and uploaded, the examples need wifi credentials
and password for Over-The-Air (OTA) code updates. These can be set by making a
new file called `wifi_secrets.h` and placing it to the same directory with the
example code. To add a new file you have to save a copy of the example to a new
location, because the examples directories are read-only.
Then, add the following content to the newly created file and change "ssid" to your
wifi name and "password" to your wifi password. Change the value of ota_password to
a password that you would like to use to secure your OTA uploads.

```c
#ifndef _SURRORTG_WIFI_SECRETS_H
#define _SURRORTG_WIFI_SECRETS_H

constexpr const char* ssid = "";            // Network name here
constexpr const char* password = "";        // Network password here
constexpr const char* ota_password = "";    // OTA-update password here

#endif  // _SURRORTG_WIFI_SECRETS_H
```

If the filename is incorrect or the file is missing, building the code
outputs an error `wifi_secrets.h: No such file or directory`.

## Development

### Code style

The project uses `clang-format` tool to make sure the code follows a specific
style. The formatting is checked on every push in the Github Actions. If the
Actions fail, please fix the formatting of the files that you have changed by
running `clang-format --style=file -i <filenames>`, where `<filenames>` are the
names of the files you want to format, separated by spaces. Alternatively you
can use a plugin on your code editor that supports `clang-format`. Remember to
commit the changes after formatting.

You can also run the following command in the root of the repository to format
all the .cpp, .h and .ino files in the repository:

```shell
find . -type f \( -name \*.cpp -o -name \*.h -o -name \*.ino \) -exec clang-format --style=file -i "{}" \;
```
