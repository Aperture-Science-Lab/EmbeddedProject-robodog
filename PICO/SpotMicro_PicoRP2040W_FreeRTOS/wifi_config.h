/**
 * @file wifi_config.h
 * @brief WiFi Configuration for SpotMicro
 * 
 * ============================================================================
 * HOW TO SET UP WIFI WITH YOUR LAPTOP'S MOBILE HOTSPOT:
 * ============================================================================
 * 
 * 1. Enable Mobile Hotspot on your Windows Laptop:
 *    - Go to Settings > Network & Internet > Mobile hotspot
 *    - Turn ON "Share my Internet connection"
 *    - Note the "Network name" (SSID) and "Network password"
 *    - Or click "Edit" to set your own name/password
 * 
 * 2. Update the credentials below:
 *    - Replace "YourHotspotName" with your hotspot's Network name
 *    - Replace "YourPassword" with your hotspot's password
 * 
 * 3. Rebuild and flash the firmware:
 *    - cd build
 *    - ninja
 *    - Copy .uf2 file to Pico
 * 
 * 4. After Pico connects:
 *    - The LCD will show the Pico's IP address
 *    - Serial monitor will also print the IP
 *    - Open browser to: http://<pico-ip>:8080
 * 
 * Example for Windows Mobile Hotspot:
 *    Network name: MyLaptop
 *    Password: 12345678
 *    Then set: WIFI_SSID "MyLaptop" and WIFI_PASSWORD "12345678"
 * 
 * ============================================================================
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// ============================================================================
// WiFi Credentials - UPDATE THESE WITH YOUR HOTSPOT INFO!
// ============================================================================
// IMPORTANT: Change these to match YOUR Windows Mobile Hotspot settings!
// Go to Windows Settings > Network & Internet > Mobile hotspot to find them
// 
// Example: If your hotspot shows:
//   Network name: DESKTOP-ABC123
//   Password: mypassword123
// Then set:
//   WIFI_SSID "DESKTOP-ABC123"
//   WIFI_PASSWORD "mypassword123"
//
#define WIFI_SSID       "CHANGE_ME"           // <-- PUT YOUR HOTSPOT NAME HERE
#define WIFI_PASSWORD   "CHANGE_ME"           // <-- PUT YOUR HOTSPOT PASSWORD HERE

#endif // WIFI_CONFIG_H
