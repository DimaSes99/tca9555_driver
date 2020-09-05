# TCA9555 gpio expander driver
## Introduction
This package contains the TCA9555 gpio expander driver

The driver package includes tca9555_driver.h, tca9555_driver.c 

## Integration details
* Integrate tca9555_driver.h, tca9555_driver.c 
* Include the tca9555_driver.h file in your code like below.
``` c
#include "tca9555_driver.h"
```

## File information
* tca9555_driver.h : This header file contains the declarations of driver APIs.
* tca9555_driver.c : This source file contains the definitions of driver APIs.


## Usage guide
### Initializing the gpio expander
To initialize gpio expander, user need to create a device structure. User can do this by 
creating an instance of the structure tca9555_dev_t. After creating the device strcuture, user 
need to fill in the various parameters as shown below.

#### Example of initialization touch controller device structure
``` c
static tca9555_dev_t gpio_expander = {
    .i2c_read = user_i2c_read,
    .i2c_write = user_i2c_write,
    .i2c_addr = tca9555_address,
};
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to
discuss what you would like to change.

Please make sure to update tests as appropriate.
