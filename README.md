# longfi-st-hal

[![Build Status](https://travis-ci.com/helium/longfi-st-hal.svg?token=bzKc8EpW7xxqudyhDiz1&branch=master)](https://travis-ci.com/helium/longfi-st-hal)

LongFi Examples Using ST HAL Libraries and [LongFi Device Protocol](https://github.com/helium/longfi-device) library.

## Board Support

### B-L072Z-LRWAN1 - ST STM32L0 Discovery kit  

[B-L072Z-LRWAN1 Product Page](https://www.st.com/en/evaluation-tools/b-l072z-lrwan1.html)  
[B-L072Z-LRWAN1 User Manual](https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/ac/62/15/c7/60/ac/4e/9c/DM00329995/files/DM00329995.pdf/jcr:content/translations/en.DM00329995.pdf)  

### Catena 4610 - MCCI  

[Catena 4610 Product Page](https://store.mcci.com/collections/iot-building-blocks/products/mcci-catena-4610-integrated-node-for-lorawan-technology)  
[Catena 4610 Pin Mapping Diagram](https://github.com/mcci-catena/HW-Designs/blob/master/Boards/Catena-4610/Catena-4610-Pinmapping.png)

## Quickstart
Visit our developer docs for a quickstart guide [here](https://developer.helium.com/device/st-hal-quickstart).

## Examples: 
* `Boards/'board name'/Examples`

## LongFi Config
In each example there is a `src/lf_radio.c` file, which contains the LongFi config needed to route packets to Helium Console.
```c
uint8_t preshared_key[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

LongFiConfig_t lf_config = {
    .oui = 1234,
    .device_id = 99,
    .auth_mode = PresharedKey128, 
};
```

## Compile
```
cd Boards/'board name'/Examples/'example name'
make
```

## Directory Layout

**`Boards`**: Collection of Board specific examples.

**`Drivers`**: Shared drivers and libraries for all example projects.

**`Drivers/CMSIS`**:  Cortex Microcontroller Software Interface Standard.

**`Drivers/longfi-device`**: Statically compiled [LongFi Device Protocol](https://github.com/helium/longfi-device) library. 

**`Drivers/STM32L0xx_HAL_Driver`**: ST HAL Driver libraries for STM32L0xx chips. 