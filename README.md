# CoolMeal
People counting Project based on i-cube-lrwan stack (STM32L072/82 - LoRaWAN EU868 ) and STSW-IMG010 stack (ToF VL53L1X)


This project is the embedded part who allows you to get a pathtrack did by a people for more information go to https://www.st.com/en/embedded-software/stsw-img010.html

Hardware:
Based on grasshopper lorawan development board powered by STM32L082 ==> https://www.tindie.com/products/tleracorp/grasshopper-loralorawan-development-board/
**********************************************************************************************************************************
To be compatible with B-L072Z-LRWAN1 (Discovery-Board) you have to change line 115 & 116 to the right mapping of TCXO.

#define RADIO_TCXO_VCC_PORT                       GPIOA
#define RADIO_TCXO_VCC_PIN                        GPIO_PIN_12
**********************************************************************************************************************************

VL53L1X (satel) ==> https://www.st.com/en/ecosystems/x-nucleo-53l1a1.html

Software:
i-cube-lrwan V1.1.5 for Keil MDK-ARM (with Time of Flight API)

Follows this requirement to increase the code size limitation compilation from 32KB to 256 KB.
https://www2.keil.com/stmicroelectronics-stm32/mdk


Mapping grasshoper:

                                               -------------------
                                               |                 |
                    ----------                 |                 |
                    |        |    INTERRUPT    |                 |
                    |        |---------------->| D5(PB2)         |--> LoRaWAN RF (EU868)
                    |  ToF   |    XSHUTDOWN    |                 |
                    |        |<----------------|AREF             |
                    |        |      3V3        |                 |
                    |        |<----------------|                 |    VIN
                    |        |      GND        |                 |<--------
                    |        |<----------------|                 |    GND
                    |        |      I2C_SCL    |                 |<--------
                    |        |<--------------->|PB8              |
                    |        |      I2C_SDA    |                 |         
                    |        |<--------------->|PB9              |
                    ----------                 |                 |
                                               |  Grassehopper   |
                                               -------------------
                                                                
                                               

Go to \CoolMeal\i-cube-lrwan_V1.1.5\STM32CubeExpansion_LRWAN_V1.1.5\Projects\Multi\Applications\LoRa\End_Node_V2.1\MDK-ARM\B-L072Z-LRWAN1\lora.uvprojx

 Add your Device EUI and lorawan connection key (api key & app key) in commissioning.h file.
 Compile your project.
 Plugin STM32L0 board and toggle the RESET button while holding down the BOOT button then from STM32CubeProgrammer load the hexfile.
 
 How does it works:
 
 The spads of Tof are divided in two Region of Interest (Front zone and Back zone) it measures alternately each Region, from that we can determine a pathtrack and increment or decrement a counter then grasshopper will send a lorawan frame (payload) thanks to a timer event (each 15s).
 
 
 It's compatible with the Things Networks (https://www.thethingsnetwork.org/) or Chirpstack server (https://www.chirpstack.io/) or others lorawan networks. 
