
boolean SdFatInitialize()
{
  boolean IsOk;
  IsOk = true;

#if (CODE_Debug > 0)
  Serial.println("SD FAT");
#endif
  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with breadboards.
  //  Use SPI_FULL_SPEED for better performance.
  IsOk = SdFatLogSystem.begin(SS_PIN, SPI_HALF_SPEED);

  // Card Size
  //  Verify that card is there
  if (IsOk)
  {
    uint32_t Size;
    uint32_t SizeMb;

    Size = SdFatLogSystem.card()->cardSize();
    if (Size == 0)
      IsOk = false;

    if (IsOk)
    {
      SizeMb = 0.000512 * Size + 0.5;  

#if (CODE_Debug == 2)
      Serial.print("   Card Size = ");
      Serial.print(SizeMb);
      Serial.println(" MB");
#endif
    }
  }

#if (CODE_Debug > 0)
  if (IsOk)
  {
    Serial.println(" Init OK");
  }
  else
  {
    SdFatLogSystem.initErrorHalt();

    Serial.println(" Init Failed!!!");
  }
#endif

  return IsOk;
}

boolean SdFatOpenWrite()
{
  boolean IsOk;
  IsOk = true;

  int FileLogNumber;

  String sFileLogName;
  char aFileLogName[10];

  String sText;

  FileLogNumber = 0;

  // Re the current Log File number
  IsOk = SdFatLogFile.open("RLN.txt", O_READ);
  if (IsOk)
  {
    FileLogNumber = SdFatLogFile.read();

    SdFatLogFile.close();
  }

  // Increment to next Log file number
  FileLogNumber ++;
  if (FileLogNumber > 9)
  {
    FileLogNumber = 1;
  }

  // Save the Log File number
  IsOk = SdFatLogFile.open("RLN.txt", (O_WRITE | O_CREAT | O_TRUNC));
  if (IsOk)
  {
    SdFatLogFile.write(FileLogNumber);

    SdFatLogFile.close();
  }

  // Open the Log File
  sFileLogName = "RL" + String(FileLogNumber) + ".txt";
  sFileLogName.toCharArray(aFileLogName, 10);

  IsOk = SdFatLogFile.open(aFileLogName, (O_WRITE | O_CREAT | O_TRUNC));
  if (IsOk)
  {
    SdFatLogFile.println("Data ...");
    SdFatLogFile.print("\r\n");

#if (CODE_Debug == 1)
    sText = "";
    sText += aFileLogName;
    sText += " Open for Write";
    Serial.println(sText);
    Serial.println("");
#endif
  }

  return IsOk;
}

int SdFatReadFileLogNumberLast()
{
  int   FileLogNumber;

  FileLogNumber = -1;

  // Re the current Log File number
  if (SdFatLogFile.open("RLN.txt", O_READ))
  {
    FileLogNumber = SdFatLogFile.read();

    SdFatLogFile.close();
  }

  return FileLogNumber;
}

void SdFatReadBack(int FileLogNumber)
{
  String sFileLogName;
  char aFileLogName[10];

  int data;

  String sText;

  sFileLogName = "RL" + String(FileLogNumber) + ".txt";
  sFileLogName.toCharArray(aFileLogName, 10);

  // Read from the file until there's nothing else in it
  if (SdFatLogFile.open(aFileLogName, O_READ))
  {
#if (CODE_Debug == 1)
    sText = "";
    sText += aFileLogName;
    sText += " Open for Read";
    Serial.println(sText);
    Serial.println("");
#endif

    // read from the file until there's nothing else in it:
    while ((data = SdFatLogFile.read()) >= 0)
    {
      Serial.write(data);
    }

    Serial.println("\r\nDone");

    // close the file:
    SdFatLogFile.close();
  }
  else
  {
    Serial.println("Read out Failed");
  }
}

















