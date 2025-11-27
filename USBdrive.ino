// USBdrive.ino
// USB Mass Storage object
#ifdef USE_TINYUSB
Adafruit_USBD_MSC usb_msc;

// Set to true when PC write to flash
bool changed = true;
bool unmount = false;
uint32_t lastChange = 0;
uint32_t  startTime = 0;
SdFile  root;
SdFile  file;

void USBdrive()
{
  USBdrive_setup();
  lastChange = millis();
  startTime = millis();
  DISP(ssd1306_setup());
  DISP(ssd1306_printBig("USB D"));
  while(!unmount) {
    if ( changed )
    {
      root.open("/");
      Serial.println("SD contents:");

      // Open next file in root.
      // Warning, openNext starts at the current directory position
      // so a rewind of the directory may be required.
      while ( file.openNext(&root, O_RDONLY) )
      {
        file.printFileSize(&Serial);
        Serial.write(' ');
        file.printName(&Serial);
        if ( file.isDir() )
        {
          // Indicate a directory.
          Serial.write('/');
        }
        Serial.println();
        file.close();
      }

      root.close();

      Serial.println();

      changed = false;
      //delay(1000); // refresh every 0.5 second
    }
    
    tud_task();
    if((millis() - startTime)>1000L) {
      if(!tud_connected()) {
        unmount = true;
      }
      if(!tud_mounted()) {
        unmount = true;
      }
    }
    if((millis() - lastChange) > 500) { 
      digitalWrite(pinRedLED,LOW);
    }
    if((millis() - lastChange)> (USBDISK_TIMEOUT*1000L)){
      Serial.println("AutoEject USB drive after no activity");
      unmount = true;
    }
    yield();
  }
  Serial.println("Unmounting USB disk");
  delay(10);
  usb_msc.setUnitReady(false);
  tud_msc_set_sense(0,SCSI_SENSE_NOT_READY,0x3A,0x00);
  Serial.println("Ending USB drive");
  lastChange = millis();
  while((millis() - lastChange)<2000) {
    tud_task();
  }
  sd.end();

  digitalWrite(pinGreenLED,LOW);
  return;
}

void USBdrive_setup() {
   // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "SD Card", "1.0");

  // Set read write callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // Still initialize MSC but tell usb stack that MSC is not ready to read/write
  // If we don't initialize, board will be enumerated as CDC only
  usb_msc.setUnitReady(false);
  usb_msc.begin();

  Serial.begin(115200);
  delay(10);
  //while ( !Serial ) delay(10);   // wait for native usb

  //Serial.println("Adafruit TinyUSB Mass Storage SD Card example");

  Serial.print("\nInitializing SD card ... ");
  Serial.print("CS = "); Serial.println(pinSDselect);

  if ( !sd.begin(pinSDselect, SD_SCK_MHZ(12)) )
  {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1) delay(1);
  }
 // Size in blocks (512 bytes)
  digitalWrite(pinGreenLED,HIGH);
  uint32_t block_count = sd.card()->sectorCount();

  Serial.print("Volume size (MB):  ");
  Serial.println((block_count/2) / 1024);

  // Set disk size, SD block size is always 512
  usb_msc.setCapacity(block_count, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);

  changed = true; // to print contents initially
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun;
  (void) power_condition;

  if ( load_eject )
  {
    if (start)
    {
      Serial.println("Loading USB Drive");
    }else
    {
      // unload disk storage
      Serial.println("Requested USB Drive Eject");
      unmount = true;
    }
  }

  return true;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  digitalWrite(pinRedLED,HIGH);
  lastChange = millis();
  return sd.card()->readSectors(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  digitalWrite(LED_BUILTIN, HIGH);
  lastChange = millis();
  return sd.card()->writeSectors(lba, buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void)
{
  sd.card()->syncDevice();
  lastChange = millis();

  // clear file system's cache to force refresh
  //sd.vol()->cacheClear();

  changed = true;

  digitalWrite(LED_BUILTIN, LOW);
}

#endif
