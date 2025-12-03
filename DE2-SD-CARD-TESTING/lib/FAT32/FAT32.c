//************************************************************
// **** ROUTINES FOR FAT32 IMPLEMATATION OF SD CARD ****
//************************************************************
//Controller		: ATmega32 (Clock: 8 Mhz-internal)
//Compiler			: AVR-GCC (winAVR with AVRStudio-4)
//Project Version	: DL_1.0
//Author			: CC Dharmani, Chennai (India)
//			  		  www.dharmanitech.com
//Date				: 10 May 2011
//************************************************************

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "FAT32.h"
// #include "UART_routines.h"
#include "uart.h"
#include "sd_routines.h"
#include "rtc.h"
#include "uart_compat.h"
// #include "RTC_routines.h"  

/**
 * @brief  Read boot sector data from SD card to determine FAT32 parameters.
 * 
 * Parses boot sector or MBR to extract bytesPerSector, sectorsPerCluster,
 * reservedSectorCount, and other critical FAT32 parameters.
 * 
 * @return 0 on success, non-zero if boot sector not found or invalid.
 */
unsigned char getBootSectorData (void)
{
struct BS_Structure *bpb; //mapping the buffer onto the structure
struct MBRinfo_Structure *mbr;
struct partitionInfo_Structure *partition;
unsigned long dataSectors;

unusedSectors = 0;

SD_readSingleBlock(0);
bpb = (struct BS_Structure *)buffer;

if(bpb->jumpBoot[0]!=0xE9 && bpb->jumpBoot[0]!=0xEB)   //check if it is boot sector
{
  mbr = (struct MBRinfo_Structure *) buffer;       //if it is not boot sector, it must be MBR
  
  if(mbr->signature != 0xaa55) return 1;       //if it is not even MBR then it's not FAT32
  	
  partition = (struct partitionInfo_Structure *)(mbr->partitionData);//first partition
  unusedSectors = partition->firstSector; //the unused sectors, hidden to the FAT
  
  SD_readSingleBlock(partition->firstSector);//read the bpb sector
  bpb = (struct BS_Structure *)buffer;
  if(bpb->jumpBoot[0]!=0xE9 && bpb->jumpBoot[0]!=0xEB) return 1; 
}

bytesPerSector = bpb->bytesPerSector;
//transmitHex(INT, bytesPerSector); transmitByte(' ');
sectorPerCluster = bpb->sectorPerCluster;
//transmitHex(INT, sectorPerCluster); transmitByte(' ');
reservedSectorCount = bpb->reservedSectorCount;
rootCluster = bpb->rootCluster;// + (sector / sectorPerCluster) +1;
firstDataSector = bpb->hiddenSectors + reservedSectorCount + (bpb->numberofFATs * bpb->FATsize_F32);

dataSectors = bpb->totalSectors_F32
              - bpb->reservedSectorCount
              - ( bpb->numberofFATs * bpb->FATsize_F32);
totalClusters = dataSectors / sectorPerCluster;
//transmitHex(LONG, totalClusters); transmitByte(' ');

if((getSetFreeCluster (TOTAL_FREE, GET, 0)) > totalClusters)  //check if FSinfo free clusters count is valid
     freeClusterCountUpdated = 0;
else
	 freeClusterCountUpdated = 1;
return 0;
}

/**
 * @brief  Calculate first sector address of a given cluster.
 * @param  clusterNumber Cluster number to convert.
 * @return First sector address of the cluster.
 */
unsigned long getFirstSector(unsigned long clusterNumber)
{
  return (((clusterNumber - 2) * sectorPerCluster) + firstDataSector);
}

/**
 * @brief  Get or set cluster entry in FAT to navigate or modify cluster chain.
 * @param  clusterNumber Current cluster number.
 * @param  get_set       GET to retrieve next cluster, SET to modify entry.
 * @param  clusterEntry  Next cluster value when get_set=SET; 0 when GET.
 * @return Next cluster number if get_set=GET; 0 if get_set=SET.
 */
unsigned long getSetNextCluster (unsigned long clusterNumber,
                                 unsigned char get_set,
                                 unsigned long clusterEntry)
{
unsigned int FATEntryOffset;
unsigned long *FATEntryValue;
unsigned long FATEntrySector;
unsigned char retry = 0;

//get sector number of the cluster entry in the FAT
FATEntrySector = unusedSectors + reservedSectorCount + ((clusterNumber * 4) / bytesPerSector) ;

//get the offset address in that sector number
FATEntryOffset = (unsigned int) ((clusterNumber * 4) % bytesPerSector);

//read the sector into a buffer
while(retry <10)
{ if(!SD_readSingleBlock(FATEntrySector)) break; retry++;}

//get the cluster address from the buffer
FATEntryValue = (unsigned long *) &buffer[FATEntryOffset];

if(get_set == GET)
  return ((*FATEntryValue) & 0x0fffffff);


*FATEntryValue = clusterEntry;   //for setting new value in cluster entry in FAT

SD_writeSingleBlock(FATEntrySector);

return (0);
}

/**
 * @brief  Get or set free cluster information from FSinfo sector.
 * @param  totOrNext    TOTAL_FREE to access free cluster count, NEXT_FREE for next free cluster.
 * @param  get_set      GET to read value, SET to update.
 * @param  FSEntry      New value when get_set=SET; 0 when GET.
 * @return Requested value if GET, 0xffffffff on error or if SET.
 */
unsigned long getSetFreeCluster(unsigned char totOrNext, unsigned char get_set, unsigned long FSEntry)
{
struct FSInfo_Structure *FS = (struct FSInfo_Structure *) &buffer;
unsigned char error;
SD_readSingleBlock(unusedSectors + 1);

if((FS->leadSignature != 0x41615252) || (FS->structureSignature != 0x61417272) || (FS->trailSignature !=0xaa550000))
  return 0xffffffff;

 if(get_set == GET)
 {
   if(totOrNext == TOTAL_FREE)
      return(FS->freeClusterCount);
   else // when totOrNext = NEXT_FREE
      return(FS->nextFreeCluster);
 }
 else
 {
   if(totOrNext == TOTAL_FREE)
      FS->freeClusterCount = FSEntry;
   else // when totOrNext = NEXT_FREE
	  FS->nextFreeCluster = FSEntry;
 
   error = SD_writeSingleBlock(unusedSectors + 1);	//update FSinfo
 }
 return 0xffffffff;
}

/**
 * @brief  Search for files/directories in root, get file address, or delete file.
 * @param  flag     GET_LIST to list directory, GET_FILE to find file, DELETE to remove.
 * @param  fileName Pointer to file name (NULL if flag=GET_LIST).
 * @return Pointer to directory entry if found, NULL otherwise.
 */
struct dir_Structure* findFiles (unsigned char flag, unsigned char *fileName)
{
// uart_puts_P("findFiles: Started\r\n");
unsigned long cluster, sector, firstSector, firstCluster, nextCluster;
struct dir_Structure *dir;
unsigned int i;
unsigned char j;
unsigned char loopCount = 0;  // ← Pridajte timeout counter

cluster = rootCluster; //root cluster
// uart_puts_P("findFiles: Root cluster = ");
// transmitHex(LONG, cluster);
// uart_puts_P("\r\n");

while(1)
{
   loopCount++;
   if(loopCount > 100) {  // ← Pridajte timeout
      //  uart_puts_P("findFiles: Loop timeout!\r\n");
       return 0;
   }
   
  //  uart_puts_P("findFiles: Reading cluster ");
  //  transmitHex(LONG, cluster);
  //  uart_puts_P("\r\n");
   
   firstSector = getFirstSector (cluster);
   
   for(sector = 0; sector < sectorPerCluster; sector++)
   {
    //  uart_puts_P("findFiles: Reading sector\r\n");
     SD_readSingleBlock (firstSector + sector);
    //  uart_puts_P("findFiles: Sector read\r\n");

     for(i=0; i<bytesPerSector; i+=32)
     {
        dir = (struct dir_Structure *) &buffer[i];

        if(dir->name[0] == EMPTY) //indicates end of the file list
        {
          uart_puts_P("findFiles: Empty entry found\r\n");
          if(flag == DELETE)
              uart_puts_P("File does not exist!");
          return 0;   
        }
		if((dir->name[0] != DELETED) && (dir->attrib != ATTR_LONG_NAME))
        {
          if((flag == GET_FILE) || (flag == DELETE))
          {
            for(j=0; j<11; j++)
            if(dir->name[j] != fileName[j]) break;
            if(j == 11)
            {
              if(flag == GET_FILE)
                    {
                appendFileSector = firstSector + sector;
              appendFileLocation = i;
              appendStartCluster = (((unsigned long) dir->firstClusterHI) << 16) | dir->firstClusterLO;
              fileSize = dir->fileSize;
                return (dir);
              }	
              else    //when flag = DELETE
              {
                TX_NEWLINE;
              uart_puts_P(("Deleting.."));
              TX_NEWLINE;
              TX_NEWLINE;
              firstCluster = (((unsigned long) dir->firstClusterHI) << 16) | dir->firstClusterLO;
                      
              //mark file as 'deleted' in FAT table
              dir->name[0] = DELETED;    
              SD_writeSingleBlock (firstSector+sector);
                    
              freeMemoryUpdate (ADD, dir->fileSize);

              //update next free cluster entry in FSinfo sector
              cluster = getSetFreeCluster (NEXT_FREE, GET, 0); 
              if(firstCluster < cluster)
                  getSetFreeCluster (NEXT_FREE, SET, firstCluster);

              //mark all the clusters allocated to the file as 'free'
                while(1)  
                {
                    nextCluster = getSetNextCluster (firstCluster, GET, 0);
                getSetNextCluster (firstCluster, SET, 0);
                if(nextCluster > 0x0ffffff6) 
                  {uart_puts_P("File deleted!");return 0;}
                firstCluster = nextCluster;
                } 
              }
            }
        }
        else  //when flag = GET_LIST
        {
            // TX_NEWLINE;
            // uart_puts_P("JUST a TEST in listing\r\n");
          for(j=0; j<11; j++)
            {
            if(j == 8) transmitByte(' ');
            // transmitByte (dir->name[j]);
          }
            uart_puts_P("   ");
            if((dir->attrib != 0x10) && (dir->attrib != 0x08))
          {
              uart_puts_P("FILE" );
                uart_puts_P("   ");
              displayMemory(LOW, dir->fileSize);
          }
          else
            {
              if (dir->attrib == 0x10)
              {
                uart_puts(("DIR" ));
              }
              else
              {
                uart_puts(("ROOT" ));
              }
            }
        }
       }
     }
   }

  //  uart_puts_P("findFiles: Getting next cluster...\r\n");
   cluster = (getSetNextCluster (cluster, GET, 0));
  //  uart_puts_P("findFiles: Next cluster = ");
  //  transmitHex(LONG, cluster);
  //  uart_puts_P("\r\n");

   if(cluster > 0x0ffffff6) {
      //  uart_puts_P("findFiles: End of cluster chain\r\n");
          return 0;
   }
   if(cluster == 0) {
      //  uart_puts_P("Error in getting cluster");
       return 0;
   }
}
return 0;
}

/**
 * @brief  Read file from SD card or verify file existence.
 * @param  flag     READ to output file contents, VERIFY to check existence.
 * @param  fileName Pointer to file name.
 * @return 0 if successful or flag=READ; 1 if file exists (VERIFY) or not found (READ); 2 if invalid filename.
 */
unsigned char readFile (unsigned char flag, unsigned char *fileName)
{
struct dir_Structure *dir;
unsigned long cluster, byteCounter = 0, fileSize, firstSector;
unsigned int k;
unsigned char j, error;

uart_puts_P("readFile: Converting filename...\r\n");
error = convertFileName (fileName); //convert fileName into FAT format
if(error) {
    uart_puts_P("readFile: Invalid filename!\r\n");
    return 2;
}

// uart_puts_P("readFile: Filename converted: ");
// for(j=0; j<11; j++) transmitByte(fileName[j]);
// uart_puts_P("\r\n");

// uart_puts_P("readFile: Calling findFiles...\r\n");
dir = findFiles (GET_FILE, fileName); //get the file location
// uart_puts_P("readFile: findFiles returned\r\n");

if(dir == 0) 
{
  // uart_puts_P("readFile: File not found\r\n");
  if(flag == READ) return (1);
  else return (0);
}

// uart_puts_P("readFile: File found!\r\n");
if(flag == VERIFY) return (1);	//specified file name is already existing

cluster = (((unsigned long) dir->firstClusterHI) << 16) | dir->firstClusterLO;

fileSize = dir->fileSize;

TX_NEWLINE;
TX_NEWLINE;

while(1)
{
  firstSector = getFirstSector (cluster);

  for(j=0; j<sectorPerCluster; j++)
  {
    SD_readSingleBlock(firstSector + j);
    
	for(k=0; k<512; k++)
    {
      // transmitByte(buffer[k]);
      if ((byteCounter++) >= fileSize ) return 0;
    }
  }
  cluster = getSetNextCluster (cluster, GET, 0);
  if(cluster == 0) {uart_puts_P("Error in getting cluster"); return 0;}
}
return 0;
}

/**
 * @brief  Convert filename from standard format to FAT 8.3 format.
 * @param  fileName Pointer to filename (input/output).
 * @return 0 on success, 1 if filename exceeds 8 characters before extension.
 */
unsigned char convertFileName (unsigned char *fileName)
{
unsigned char fileNameFAT[11];
unsigned char j, k;

uart_puts_P("cvt: start\r\n");

for(j=0; j<12; j++)
  if(fileName[j] == '.') break;

uart_puts_P("cvt: dot at ");
transmitByte(j + '0');
uart_puts_P("\r\n");

if(j>8) {
  uart_puts_P("cvt: name too long\r\n");
  return 1;
}

for(k=0; k<j; k++)
  fileNameFAT[k] = fileName[k];

for(k=j; k<=7; k++)
  fileNameFAT[k] = ' ';

uart_puts_P("cvt: name done\r\n");

j++;
for(k=8; k<11; k++)
{
  if(fileName[j] != 0)
    fileNameFAT[k] = fileName[j++];
  else
    while(k<11)
      fileNameFAT[k++] = ' ';
}

uart_puts_P("cvt: ext done\r\n");

for(j=0; j<11; j++)
  if((fileNameFAT[j] >= 0x61) && (fileNameFAT[j] <= 0x7a))
    fileNameFAT[j] -= 0x20;

uart_puts_P("cvt: caps done\r\n");

for(j=0; j<11; j++)
  fileName[j] = fileNameFAT[j];

uart_puts_P("cvt: OK\r\n");

return 0;
}

/**
 * @brief  Create new file in FAT32 format or append to existing file.
 * 
 * Creates file in root directory with proper directory entry and cluster chain.
 * Updates FSinfo sector with free cluster information.
 * 
 * @param  fileName Pointer to filename.
 * @return 0 on success, 1 on failure (no free clusters or invalid filename).
 */
unsigned char writeFile (unsigned char *fileName)
{
uart_puts_P("writeFile: Started\r\n");
unsigned char j,k, data, error, fileCreatedFlag = 0, start = 0, appendFile = 0, sector=0;
unsigned int i, firstClusterHigh=0, firstClusterLow=0;
struct dir_Structure *dir;
unsigned long cluster, nextCluster, prevCluster, firstSector, clusterCount, extraMemory;

uart_puts_P("writeFile: Verifying file...\r\n");
j = readFile (VERIFY, fileName);
uart_puts_P("writeFile: Verify result = ");
transmitByte(j + '0');
uart_puts_P("\r\n");

if(j == 1) 
{
  uart_puts_P("writeFile: File exists, appending...\r\n");
  appendFile = 1;
  cluster = appendStartCluster;
  clusterCount=0;
  while(1)
  {
    nextCluster = getSetNextCluster (cluster, GET, 0);
    if(nextCluster == EOF) break;
  cluster = nextCluster;
  clusterCount++;
  }
  uart_puts_P("writeFile: Cluster count = ");
  transmitHex(LONG, clusterCount);
  uart_puts_P("\r\n");

  sector = (fileSize - (clusterCount * sectorPerCluster * bytesPerSector)) / bytesPerSector;
  start = 1;
}
else if(j == 2) 
{
   uart_puts_P("writeFile: Invalid filename\r\n");
   return 1;
}
else
{
  uart_puts_P("writeFile: Creating new file...\r\n");
  cluster = getSetFreeCluster (NEXT_FREE, GET, 0);
  uart_puts_P("writeFile: Next free cluster = ");
  transmitHex(LONG, cluster);
  uart_puts_P("\r\n");
  
  if(cluster > totalClusters)
     cluster = rootCluster;

  cluster = searchNextFreeCluster(cluster);
  if(cluster == 0)
  {
    uart_puts_P("writeFile: No free cluster!\r\n");
    return 1;
  }
  uart_puts_P("writeFile: Using cluster = ");
  transmitHex(LONG, cluster);
  uart_puts_P("\r\n");
  
  getSetNextCluster(cluster, SET, EOF);
  firstClusterHigh = (unsigned int) ((cluster & 0xffff0000) >> 16 );
  firstClusterLow = (unsigned int) ( cluster & 0x0000ffff);
  fileSize = 0;
}

k=0;
uart_puts_P("writeFile: Writing data...\r\n");

while(1)
{
   if(start)
   {
      start = 0;
    startBlock = getFirstSector (cluster) + sector;
    SD_readSingleBlock (startBlock);
    i = fileSize % bytesPerSector;
    j = sector;
   }
   else
   {
      startBlock = getFirstSector (cluster);
    i=0;
    j=0;
   }
   
   do
   {
   data = dataString[k++];
     buffer[i++] = data;
   fileSize++;
     
     if(i >= 512)
   {
     i=0;
     error = SD_writeSingleBlock (startBlock);
       j++;
     if(j == sectorPerCluster) {j = 0; break;}
     startBlock++; 
     }
   } while((data != '\n') && (k < MAX_STRING_SIZE));

   if((data == '\n') || (k >= MAX_STRING_SIZE))
   {
      uart_puts_P("writeFile: End of data\r\n");
      for(;i<512;i++)
        buffer[i]= 0x00;
      error = SD_writeSingleBlock (startBlock);
      break;
   } 
 
   prevCluster = cluster;
   cluster = searchNextFreeCluster(prevCluster);

   if(cluster == 0)
   {
      uart_puts_P("writeFile: No free cluster during write!\r\n");
    return 1;
   }

   getSetNextCluster(prevCluster, SET, cluster);
   getSetNextCluster(cluster, SET, EOF);
}        

getSetFreeCluster (NEXT_FREE, SET, cluster);

error = getDateTime_FAT();
if(error) { dateFAT = 0; timeFAT = 0;}

if(appendFile)
{
  uart_puts_P("writeFile: Updating directory entry...\r\n");
  SD_readSingleBlock (appendFileSector);    
  dir = (struct dir_Structure *) &buffer[appendFileLocation]; 

  dir->lastAccessDate = 0;
  dir->writeTime = timeFAT;
  dir->writeDate = dateFAT;
  extraMemory = fileSize - dir->fileSize;
  dir->fileSize = fileSize;
  SD_writeSingleBlock (appendFileSector);
  freeMemoryUpdate (REMOVE, extraMemory);

  uart_puts_P("writeFile: File appended successfully\r\n");
  return 0;
}

//executes following portion when new file is created

prevCluster = rootCluster; //root cluster

while(1)
{
   firstSector = getFirstSector (prevCluster);

   for(sector = 0; sector < sectorPerCluster; sector++)
   {
     SD_readSingleBlock (firstSector + sector);
	

     for(i=0; i<bytesPerSector; i+=32)
     {
	    dir = (struct dir_Structure *) &buffer[i];

		if(fileCreatedFlag)   //to mark last directory entry with 0x00 (empty) mark
		 { 					  //indicating end of the directory file list
		   //dir->name[0] = EMPTY;
		   //SD_writeSingleBlock (firstSector + sector);
           return 0;
         }

        if((dir->name[0] == EMPTY) || (dir->name[0] == DELETED))  //looking for an empty slot to enter file info
		{
		  for(j=0; j<11; j++)
  			dir->name[j] = fileName[j];
		  dir->attrib = ATTR_ARCHIVE;	//settting file attribute as 'archive'
		  dir->NTreserved = 0;			  //always set to 0
		  dir->timeTenth = 0;			    //always set to 0
		  dir->createTime = timeFAT; 	//setting time of file creation, obtained from RTC
		  dir->createDate = dateFAT; 	//setting date of file creation, obtained from RTC
		  dir->lastAccessDate = 0;   	//date of last access ignored
		  dir->writeTime = timeFAT;  	//setting new time of last write, obtained from RTC
		  dir->writeDate = dateFAT;  	//setting new date of last write, obtained from RTC
		  dir->firstClusterHI = firstClusterHigh;
		  dir->firstClusterLO = firstClusterLow;
		  dir->fileSize = fileSize;

		  SD_writeSingleBlock (firstSector + sector);
		  fileCreatedFlag = 1;

		  //TX_NEWLINE;
		  //TX_NEWLINE;
		  //uart_puts_P(PSTR(" File Created! "));

		  freeMemoryUpdate (REMOVE, fileSize); //updating free memory count in FSinfo sector
	     
        }
     }
   }

   cluster = getSetNextCluster (prevCluster, GET, 0);

   if(cluster > 0x0ffffff6)
   {
      if(cluster == EOF)   //this situation will come when total files in root is multiple of (32*sectorPerCluster)
	  {  
		cluster = searchNextFreeCluster(prevCluster); //find next cluster for root directory entries
		getSetNextCluster(prevCluster, SET, cluster); //link the new cluster of root to the previous cluster
		getSetNextCluster(cluster, SET, EOF);  //set the new cluster as end of the root directory
      } 

      else
      {	
	    uart_puts_P("End of Cluster Chain"); 
	    return 1;
      }
   }
   if(cluster == 0) {uart_puts_P("Error in getting cluster"); return 1;}
   
   prevCluster = cluster;
 }
 
 return 0;
}


/**
 * @brief  Search for next free cluster in FAT starting from specified cluster.
 * @param  startCluster Starting cluster number.
 * @return First free cluster found, 0 if no free clusters available.
 */
unsigned long searchNextFreeCluster (unsigned long startCluster)
{
  unsigned long cluster, *value, sector;
  unsigned char i;
    
	startCluster -=  (startCluster % 128);   //to start with the first file in a FAT sector
    for(cluster =startCluster; cluster <totalClusters; cluster+=128) 
    {
      sector = unusedSectors + reservedSectorCount + ((cluster * 4) / bytesPerSector);
      SD_readSingleBlock(sector);
      for(i=0; i<128; i++)
      {
       	 value = (unsigned long *) &buffer[i*4];
         if(((*value) & 0x0fffffff) == 0)
            return(cluster+i);
      }  
    } 

 return 0;
}

/**
 * @brief  Display memory size as formatted text string on UART.
 * @param  flag   HIGH for display in KBytes, LOW for display in Bytes.
 * @param  memory Memory value to display.
 * @return none
 */
void displayMemory (unsigned char flag, unsigned long memory)
{
  unsigned char memoryString[] = "              Bytes"; //19 character long string for memory display
  unsigned char i;
  for(i=12; i>0; i--) //converting freeMemory into ASCII string
  {
    if(i==5 || i==9) 
	{
	   memoryString[i-1] = ',';  
	   i--;
	}
    memoryString[i-1] = (memory % 10) | 0x30;
    memory /= 10;
	if(memory == 0) break;
  }
  if(flag == HIGH)  memoryString[13] = 'K';
  transmitString(memoryString);
}

/**
 * @brief  Delete specified file from root directory.
 * @param  fileName Pointer to filename to delete.
 * @return none
 */
void deleteFile (unsigned char *fileName)
{
  unsigned char error;

  error = convertFileName (fileName);
  if(error) return;

  findFiles (DELETE, fileName);
}

/**
 * @brief  Update free cluster count in FSinfo sector.
 * 
 * Called when files are created or deleted to maintain accurate free space info.
 * 
 * @param  flag ADD to increase free clusters, REMOVE to decrease.
 * @param  size File size in bytes (converted to clusters internally).
 * @return none
 */
void freeMemoryUpdate (unsigned char flag, unsigned long size)
{
  unsigned long freeClusters;
  //convert file size into number of clusters occupied
  if((size % 512) == 0) size = size / 512;
  else size = (size / 512) +1;
  if((size % 8) == 0) size = size / 8;
  else size = (size / 8) +1;

  if(freeClusterCountUpdated)
  {
	freeClusters = getSetFreeCluster (TOTAL_FREE, GET, 0);
	if(flag == ADD)
  	   freeClusters = freeClusters + size;
	else  //when flag = REMOVE
	   freeClusters = freeClusters - size;
	getSetFreeCluster (TOTAL_FREE, SET, freeClusters);
  }
}

//******** END ****** www.dharmanitech.com *****