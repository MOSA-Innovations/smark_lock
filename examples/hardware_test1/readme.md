## To flash the firmware:

```bash
nrfjprog -f nrf52 --program hex/custompcb_twi_scanner_pca10040.hex --sectorerase
nrfjprog -f nrf52 --reset
```
## To debug
Use a serial terminal connected to IO.06 (eg. sudo picocom -b 115200 /dev/ttyUSB0): 
```bash
<info> app: Testing custom PCB.
<info> app: Scanning for TWI devices.
<info> app: TWI device detected at address 0x68.
<info> app: Set the POWER Control PIN(IO19,IO20) High and Low.
<info> app: Set the POWER Control PIN(IO19,IO20) Low and High.
<info> app: Set the POWER Control PIN(IO19,IO20) High and High.
```

Debbuging:
Poor connection of I2C devices might pause the scanning process.
