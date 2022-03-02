# DESCENDERE-Payload

## 💾 Upload

1. Navigate to `src`/`main.cpp`
2. Change the file name from `main.cpp` to `src.ino`
3. Double-click (open with Arduino IDE)
4. Select Teensy 4.0 as the board
5. Upload

## 🛠️ Fixes/Improvements from P'Pop's codebase:

-   Use `EEPROM.update()` where available to save EEPROM writecycle.
-   Use `EEPROM.get()` instead of `read` function to read multi-byte data.
-   Save ground altitude to EEPROM for reference only on mission start and not on recovery.
-   Avoid unnecessary code duplication in file name determination.
-   Combine packet-related variables into struct (_Experimental_)
