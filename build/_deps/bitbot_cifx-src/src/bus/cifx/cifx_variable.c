#include "bitbot_cifx/bus/cifx/cifx_driver.h"

#include <unistd.h>
#include <cifx/cifXErrors.h>

CIFXHANDLE cifx_driver = NULL;
CIFXHANDLE cifx_channel = NULL;
int32_t	 cifx_return_val = CIFX_NO_ERROR;

