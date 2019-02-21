/* smbus_functions.h */
/* I found that i2c-dev.h in 16.04 + libi2c-dev includes the i2c_smbus_... functions
 * and 18.04 + libi2c-dev does not include them. In the interest of avoiding these
 * shenanigans and make a ROS package that painlessly "just works" I make a local copy of all
 * those functions (which are really just inline functions using ioctl) here. */

#ifndef _smbus_functions_dot_h
#define _smbus_functions_dot_h

#include <linux/types.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

/* This is the structure as used in the I2C_SMBUS ioctl call */
struct _i2c_smbus_ioctl_data {
	char read_write;
	__u8 command;
	int size;
	union i2c_smbus_data *data;
};

/* This is the structure as used in the I2C_RDWR ioctl call */
struct _i2c_rdwr_ioctl_data {
	struct i2c_msg *msgs;	/* pointers to i2c_msgs */
	int nmsgs;		/* number of i2c_msgs */
};

static inline __s32 _i2c_smbus_access(int file, char read_write, __u8 command, 
                                     int size, union i2c_smbus_data *data)
{
	struct _i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}


static inline __s32 _i2c_smbus_write_quick(int file, __u8 value)
{
	return _i2c_smbus_access(file,value,0,I2C_SMBUS_QUICK,NULL);
}
	
static inline __s32 _i2c_smbus_read_byte(int file)
{
	union i2c_smbus_data data;
	if (_i2c_smbus_access(file,I2C_SMBUS_READ,0,I2C_SMBUS_BYTE,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

static inline __s32 _i2c_smbus_write_byte(int file, __u8 value)
{
	return _i2c_smbus_access(file,I2C_SMBUS_WRITE,value,
	                        I2C_SMBUS_BYTE,NULL);
}

static inline __s32 _i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (_i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

static inline __s32 _i2c_smbus_write_byte_data(int file, __u8 command, 
                                              __u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return _i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_BYTE_DATA, &data);
}

static inline __s32 _i2c_smbus_read_word_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (_i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_WORD_DATA,&data))
		return -1;
	else
		return 0x0FFFF & data.word;
}

static inline __s32 _i2c_smbus_write_word_data(int file, __u8 command, 
                                              __u16 value)
{
	union i2c_smbus_data data;
	data.word = value;
	return _i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_WORD_DATA, &data);
}

static inline __s32 _i2c_smbus_process_call(int file, __u8 command, __u16 value)
{
	union i2c_smbus_data data;
	data.word = value;
	if (_i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
	                     I2C_SMBUS_PROC_CALL,&data))
		return -1;
	else
		return 0x0FFFF & data.word;
}


/* Returns the number of read bytes */
static inline __s32 _i2c_smbus_read_block_data(int file, __u8 command, 
                                              __u8 *values)
{
	union i2c_smbus_data data;
	int i;
	if (_i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BLOCK_DATA,&data))
		return -1;
	else {
		for (i = 1; i <= data.block[0]; i++)
			values[i-1] = data.block[i];
			return data.block[0];
	}
}

static inline __s32 _i2c_smbus_read_i2c_block_data(int file, __u8 command,
   __u8 length, __u8 *values)
{
	union i2c_smbus_data data;

	if (length > 32) {
		length = 32;
	}

	data.block[0] = length;
	if (_i2c_smbus_access(file, I2C_SMBUS_READ, command
			, length == 32 ? I2C_SMBUS_I2C_BLOCK_BROKEN : I2C_SMBUS_I2C_BLOCK_DATA
			, &data))
	{
			return -1;
	} else {
			for (int i = 1; i <= data.block[0]; i++) {
					values[i - 1] = data.block[i];
			}

			return data.block[0];
	}
}

static inline __s32 _i2c_smbus_write_block_data(int file, __u8 command, 
                                               __u8 length, __u8 *values)
{
	union i2c_smbus_data data;
	int i;
	if (length > 32)
		length = 32;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return _i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_BLOCK_DATA, &data);
}

static inline __s32 _i2c_smbus_write_i2c_block_data(int file, __u8 command,
                                               __u8 length, __u8 *values)
{
	union i2c_smbus_data data;
	int i;
	if (length > 32)
		length = 32;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return _i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_I2C_BLOCK_DATA, &data);
}

#endif // _smbus_functions_dot_h
