/*
      24BIT REGISTER MAP FOR MCP3912
      
      The READ bits <1:0> in the STATUSCOM register select continuous read mode
        00  =  Static (no incrementation)
        01  =  Goups
        10  =  Types (default)
        11  =  Full Register map
      In any mode, the read will wrap around on the register list, beginning with the register spedified in the Control Byte
      
*/

#define DEACTIVATE 0
#define ACTIVATE 1
#define ENABLE_0  0x000E0000
#define ENABLE_1  0x000D0000
#define ENABLE_2  0x000B0000
#define ENABLE_3  0x00070000
#define DISABLE_0  0x00010000
#define DISABLE_1  0x00020000
#define DISABLE_2  0x00040000
#define DISABLE_3  0x00080000


#define _SCLK  4  // make sure these pins are also set in variants.h file
#define _MISO  3
#define _MOSI  2

#define MCP_SS	5  // MCP Slave Select on RFduiono pin 5
#define DR	6  // DataReady on RFduino pin 6

#define MCP_ADD  0x40    
#define MCP_READ  0x01  
#define MCP_WRITE  0x00  

#define DEV_ADD  0x40

#define CHAN_0  0x00       //00   |        | -GROUP
#define CHAN_1  0x02       //01   |-TYPE  _|
#define CHAN_2  0x04       //02   |        | -GROUP
#define CHAN_3  0x06       //03  _|       _|
 
#define MOD_VAL  0x10      //08   |        |
#define PHASE  0x14        //0A   |        |-GROUP
#define GAIN  0x16         //0B   |       _|
#define STATUSCOM  0x18    //0C   |        |
#define CONFIG_0  0x1A     //0D   |        |-GROUP
#define CONFIG_1  0x1C     //0E   |       _|
#define OFFCAL_0  0x1E     //0F   |-TYPE   | -GROUP
#define GAINCAL_0  0x20    //10   |       _|
#define OFFCAL_1  0x22     //11   |        | -GROUP
#define GAINCAL_1  0x24    //12   |       _|
#define OFFCAL_2  0x26     //13   |        | -GROUP
#define GAINCAL_2  0x28    //14   |       _|
#define OFFCAL_3  0x2A     //15   |        | -GROUP
#define GAINCAL_3  0x2C    //16   |       _|
#define LOK_CRC  0x3E      //1F  _|        >-GROUP

#define GAIN_1  0x00000000
#define GAIN_2  0x00000249
#define GAIN_4  0x00000492
#define GAIN_8  0x000006CB
#define GAIN_16 0x00000924
#define GAIN_32 0x00000B6D

#define CONFIG_8MHZ_256  0x0039E050
#define CONFIG_8MHZ_512  0x0038E050
#define CONFIG_4MHZ_128  0x0039E050
#define CONFIG_4MHZ_256  0x0038E050


