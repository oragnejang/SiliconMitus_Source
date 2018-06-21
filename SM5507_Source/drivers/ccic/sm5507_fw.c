
#include <linux/fs.h>
#include <linux/ccic/sm5507.h>
#include <linux/uaccess.h>
#include <linux/ccic/BOOT_FLASH_FW_SM5507.h>




int hex_to_int(const char *hexa)
{
    int deci = 0;
    const char *sp = hexa;
    char ch = 0;
    
    while(*sp){
        deci *= 16;
        ch = 0;        
        if ( ('0' <= *sp) && ( *sp <='9') )
            ch = *sp - '0';
        if ( ('A' <= *sp) && ( *sp <='F') )
            ch = *sp -'A' + 10;
        if ( ('a' <= *sp) && ( *sp <='f') )
            ch = *sp -'a' + 10;
        deci += ch;
        sp++;
    }
    return deci;
}


void sm5507_get_fw_version(struct sm5507_version *version, u8 boot_version)
{
    /* To-Do : get_version_from_header_file */
    version->main = FIRMWARE_VERSION_SM5507[0];
    version->boot = FIRMWARE_VERSION_SM5507[1];
}

void sm5507_get_chip_swversion(struct sm5507_data *data, struct sm5507_version *version)
{
    struct i2c_client *i2c = data->i2c;

    version->boot = sm5507_i2c_read_byte(i2c, 0x2A); // boot version
    version->main = sm5507_i2c_read_byte(i2c, 0x2B); // main version


}

int sm5507_flash_write_byte(const struct i2c_client *i2c, u16 reg, u8 *val, u16 size)
{
	int ret = 0; u8 buf[258] = {0,};
	struct i2c_msg msg[1];
	struct sm5507_data *usbpd_data = i2c_get_clientdata(i2c);

	if (size > 256)
	{
		pr_err("I2C error, over the size %d", size);
		return -EIO;
	}

	mutex_lock(&usbpd_data->i2c_mutex);
	msg[0].addr = i2c->addr;
	msg[0].flags = 0;
	msg[0].len = size+2;
	msg[0].buf = buf;

	buf[0] = (reg & 0xFF00) >> 8;
	buf[1] = (reg & 0xFF);
	memcpy(&buf[2], val, size);

	ret = i2c_transfer(i2c->adapter, msg, 1);
	if (ret < 0)
		dev_err(&i2c->dev, "i2c write fail reg:0x%x error %d\n", reg, ret);
	mutex_unlock(&usbpd_data->i2c_mutex);

	return ret;
}


static int sm5507_flash_write(struct sm5507_data *data, unsigned char *fw_data)
{
    struct i2c_client *i2c = data->i2c;
    int i;
    int buffer_address = 0;


    for(i=0; i<512; i++)
    {
        sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_CTRL, 0x01); // Set Page Load Mode

        // Set Page Load Address
        sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_ADDR0, 0x00);
        sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_ADDR1, 0x00); 

        // Write Page Data
        sm5507_flash_write_byte(i2c, SM5507_REG_MTP_WDATA, &fw_data[buffer_address], 64);
        
        sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_CTRL, 0x00); // Clear CMD

        // Set Page Address for Program
        sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_ADDR0, (buffer_address&0x00FF));
        sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_ADDR1, ((buffer_address&0x7F00)>>8));

        sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_CTRL, 0x08); // Command Program
        msleep(5); // Wait 5msec

        sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_CTRL, 0x00); // Clear CMD

        buffer_address += 64;
    }
    return 0;
}

int sm5507_flash(struct sm5507_data *data, unsigned int input)
{
	struct i2c_client *i2c = data->i2c;
    u8 W_DATA[2];//, flash_enter_mode;
    uint8_t *MTP_Buffer;
    char input_hex[3] = {0,0,0};
    int ret = 0, temp_data = 0, input_index = 0;
	mm_segment_t old_fs;
	struct file *fp;
    long fw_size, nread, count;

    switch (input) {
        case FLASH_MODE_ENTER:
            W_DATA[0] = 0xF5;
            W_DATA[1] = 0x9E;

            sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_TMODE, W_DATA[0]);
            sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_TMODE, W_DATA[1]);
            break;
        case FLASH_ERASE:
//            PD_REQ = sm5507_i2c_read_byte(i2c, SM5507_REG_PD_REQ);            
            sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_IUM, 0x00); // Select main area
            sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_CTRL, 0x34); // All Erase MTP
            msleep(10);
            sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_CTRL, 0x00); // Clear CMD
            break;
        case FLASH_WRITE:
//            flash_enter_mode = sm5507_i2c_read_byte(i2c, SM5507_REG_MTP_TMODE);
            sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_IUM, 0x00); // Select main area
            sm5507_flash_write(data, (unsigned char*)&BOOT_FLASH_FW_SM5507[0]);
            break;
        case FLASH_WRITE_UMS:
            sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_IUM, 0x00); // Select main area

    		old_fs = get_fs();
	    	set_fs(KERNEL_DS);
		    fp = filp_open(CCIC_DEFAULT_UMS_SM5507_FW, O_RDONLY, S_IRUSR);

    		if (IS_ERR(fp)) {
	    		pr_err("%s: failed to open %s.\n", __func__,
		    		      CCIC_DEFAULT_UMS_SM5507_FW);
			    ret = -ENOENT;
			    goto done;
		    }

    		fw_size = fp->f_path.dentry->d_inode->i_size;
	    	if (0 < fw_size) {
		    	unsigned char *fw_data;
		    	fw_data = kzalloc(fw_size, GFP_KERNEL);
	    		nread = vfs_read(fp, (char __user *)fw_data,
    					 fw_size, &fp->f_pos);                
 			    pr_err("%s: start, file path %s, size %ld Bytes\n",
				           __func__, CCIC_DEFAULT_UMS_SM5507_FW, fw_size);
            
			    if (nread != fw_size) {
				    pr_err("%s: failed to read firmware file, nread %ld Bytes\n",
					      __func__, nread);
    				ret = -EIO;
	    		} else {
                    count = 0;
                    MTP_Buffer = kzalloc(nread, GFP_KERNEL);
                    while(1)  {
                        if(nread == count)
                            break;

                        input_hex[0] = fw_data[count];
                        input_hex[1] = fw_data[count+1];
                        input_hex[2] = '\0';
                        temp_data = hex_to_int(input_hex);
    
                        MTP_Buffer[input_index] = temp_data;
                        input_index++;
                        count += 2;
                    }
                    sm5507_flash_write(data, &MTP_Buffer[0]);
                    kfree(MTP_Buffer);
                }

                kfree(fw_data);
            }
	    	filp_close(fp, NULL);
    		set_fs(old_fs);            
            break;
        case FLASH_MODE_EXIT:
            sm5507_i2c_write_byte(i2c, SM5507_REG_MTP_TMODE, 0x00);            
            break;
    }
done:
	return ret;    
}

int sm5507_flash_fw(struct sm5507_data *data, unsigned int input)
{
//	struct i2c_client *i2c = data->i2c;

	pr_err("FW_UPDATE %d\n", input);

    switch (input) {
        case FLASH_WRITE:
            sm5507_flash(data, FLASH_MODE_ENTER);
            sm5507_flash(data, FLASH_ERASE);
            sm5507_flash(data, input);            
            sm5507_flash(data, FLASH_MODE_EXIT);            
            break;
        case FLASH_WRITE_UMS:
            disable_irq(data->irq);
            // LPM OFF ?
            sm5507_flash(data, FLASH_MODE_ENTER);
            sm5507_flash(data, FLASH_ERASE);
            sm5507_flash(data, input);            
            sm5507_flash(data, FLASH_MODE_EXIT);            
            // LPM ON ?
            enable_irq(data->irq);
            break;
        default:
            break;
    }
    return 0;
}

