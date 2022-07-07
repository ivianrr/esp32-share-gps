# esp32-share-gps
This is an example sketch that shows how to retrieve GPS information with the ESP32 from a smartphone. 

For this to work, you need to install [Share GPS](https://play.google.com/store/apps/details?id=com.jillybunch.shareGPS&hl=en) in your Android device first. 

Then, you need to enable your Android device Wi-Fi Hotspot and replace your credentials accordingly in `share-gps.ino`:

    const char* ssid     = "***";     
    const char* password = "***"; 

Remember to update the IP address of your phone as well.
    const IPAddress server = IPAddress(192, 168, 52, 141);  // Server IP address

In the app, add a GPSD connection (leave the default port as 2947) and click on it to enable it. It should turn from `Idle` to `Listening`.
For more info about setting up the app, [this repo](https://github.com/franckalbinet/iot-uaa-isoc/blob/master/labs/lora-coverage.md) can be helpful even though it is intended for LoPy.

After flashing the ESP32, the serial monitor should start displaying the GPS coordinates of the phone and a some debug information.
