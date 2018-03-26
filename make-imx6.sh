#!/bin/sh

image_filename="u-boot.imx"
target_filename="u-boot.imx.btds"


if [ ! -f include/autoconf.mk ]; then
	echo "...mx6dual BTDS config"
	make  arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- mx6qbtds_config
fi

if [ "$1" = "" ]; then
	ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- 
else
    ARCH=arm make arm=ARM CROSS_COMPILE=arm-linux-gnueabihf- $1 $2 $3
fi

if [ -f $image_filename ]; then
   echo "copy from $image_filename to /tftpboot/$target_filename"
   cp  $image_filename /tftpboot/$target_filename
   chmod 777 /tftpboot/$target_filename
fi
