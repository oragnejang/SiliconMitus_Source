
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h> 
#include <linux/uaccess.h> 
#include <linux/ccic/ccic_sysfs.h>
#include <linux/ccic/sm5507.h>

extern int sm5507_flash_fw(struct sm5507_data *data, unsigned int input);

static int sm5507_firmware_update_built_in(struct device *dev)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);
	struct sm5507_version chip_swver, fw_swver;//, hwver;

	sm5507_get_chip_swversion(sm5507_data, &chip_swver);
	pr_err("%s CHIP SWversion %2x, %2x - before\n", __func__,
	       chip_swver.main, chip_swver.boot);
	sm5507_get_fw_version(&fw_swver, chip_swver.boot);
	pr_err("%s SRC SWversion:%2x, %2x\n",__func__,
		fw_swver.main, fw_swver.boot);

	pr_err("%s: FW UPDATE boot:%01d\n", __func__, chip_swver.boot);

    //Need to version check
    if(chip_swver.main < fw_swver.main)
    {
        sm5507_flash_fw(sm5507_data, FLASH_WRITE);
    }


	return 0;
}

static int sm5507_firmware_update_ums(struct device *dev)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);
	int error = 0;
	unsigned char *fw_data;
    struct sm5507_version fw_swver;
	struct file *fp;    
    long fw_size, nread;
    mm_segment_t old_fs;

	if(!sm5507_data) {
		pr_err("%s sm5507_data is null!!\n", __func__);
		return -ENODEV;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(CCIC_DEFAULT_UMS_SM5507_FW, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		pr_err("%s: failed to open %s.\n", __func__,
						CCIC_DEFAULT_UMS_SM5507_FW);
		error = -ENOENT;
		goto open_err;
	}

	fw_size = fp->f_path.dentry->d_inode->i_size;

	if (0 < fw_size) {
		fw_data = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);

		pr_info("%s: start, file path %s, size %ld Bytes\n",
					__func__, CCIC_DEFAULT_UMS_SM5507_FW, fw_size);
		filp_close(fp, NULL);

		if (nread != fw_size) {
			pr_err("%s: failed to read firmware file, nread %ld Bytes\n",
					__func__, nread);
			error = -EIO;
		} else {
		    fw_swver.boot = fw_data[nread -2];
   		    fw_swver.main = fw_data[nread -1];
			pr_info("CCIC FW ver - cur:%02X %02X / bin:%02X %02X\n",
					sm5507_data->firm_ver[2], sm5507_data->firm_ver[3],
					fw_swver.boot, fw_swver.main);		
            if(fw_swver.boot == sm5507_data->firm_ver[3]) {
                if (sm5507_flash_fw(sm5507_data, FLASH_WRITE_UMS) >=0)
                    goto done;
            } else {
				pr_err("error : Didn't match to CCIC FW firmware version\n");
				error = -EINVAL;            
            }
        }
   		if (error < 0)
			pr_err("%s: failed update firmware\n", __func__);
done:
    kfree(fw_data);        
    }
open_err:
    set_fs(old_fs);
    return error;

}
static ssize_t sm5507_cur_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);
	struct sm5507_version chip_swver;

	sm5507_get_chip_swversion(sm5507_data, &chip_swver);
	pr_err("%s CHIP SWversion %2x, %2x\n", __func__,
	       chip_swver.main, chip_swver.boot);

	sm5507_data->firm_ver[2] = chip_swver.main;
	sm5507_data->firm_ver[3] = chip_swver.boot;

	return sprintf(buf, "%02X, %02X\n", sm5507_data->firm_ver[2], sm5507_data->firm_ver[3]);

}
static DEVICE_ATTR(cur_version, 0444, sm5507_cur_version_show, NULL);

static ssize_t sm5507_src_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);
	struct sm5507_version fw_swver;

	sm5507_get_fw_version(&fw_swver, sm5507_data->firm_ver[3]);
	return sprintf(buf, "%02X, %02X\n",
		fw_swver.main, fw_swver.boot);

}
static DEVICE_ATTR(src_version, 0444, sm5507_src_version_show, NULL);

static ssize_t sm5507_lpm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sm5507_data *sms5507_data = dev_get_drvdata(dev);


    if (!sms5507_data) {
        pr_err("sms5507_data is NULL\n");
        return -ENODEV;
    }

    return sprintf(buf, "%d\n", sms5507_data->manual_lpm_mode);

}
static ssize_t sm5507_lpm_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);
	int mode;

    if (!sm5507_data) {
        pr_err("usbpd_data is NULL\n");
        return -ENODEV;
    }

	sscanf(buf, "%d", &mode);
	pr_info("usb: %s mode=%d\n", __func__, mode);

    // manual_JIGON - SYS_CTRL(0x1B)
    // manual_LPM - PWR_REQ(0x43)
    
	switch(mode){
	case 0:
		/* Disable Low Power Mode for App (JIGON Low + LP Off) */
  		sm5507_manual_LPM(sm5507_data, 1);
        sm5507_manual_JIGON(sm5507_data, 0);
        sm5507_data->manual_lpm_mode = 0;
        break;
    case 1:
		/* Enable Low Power Mode (JIGON High + Force LP On) */        
        sm5507_manual_JIGON(sm5507_data, 1);
        sm5507_manual_LPM(sm5507_data, 0);
        sm5507_data->manual_lpm_mode = 1;
        break;
    case 2:
		/* Enable Low Power Mode (Normal LP On) */
        sm5507_manual_JIGON(sm5507_data, 1);
        sm5507_manual_LPM(sm5507_data, 0);
        sm5507_data->manual_lpm_mode = 1;
        break;
    case 3:
		/* Disable Low Power Mode (LP Off) */
        sm5507_manual_LPM(sm5507_data, 1);
        sm5507_data->manual_lpm_mode = 0;
        break;
    default:
		/* Disable Low Power Mode (JIGON Low + LP Off) */
  		sm5507_manual_LPM(sm5507_data, 1);
        sm5507_manual_JIGON(sm5507_data, 0);        
        sm5507_data->manual_lpm_mode = 0;
        break;
    }
    return size;
}
static DEVICE_ATTR(lpm_mode, 0664, 
	sm5507_lpm_mode_show, sm5507_lpm_mode_store);

static ssize_t sm5507_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);

	if(!sm5507_data) {
		pr_err("%s sm5507_data is null!!\n", __func__);
		return -ENODEV;
	}

    return sprintf(buf, "%d\n", sm5507_data->pd_state);
}
static DEVICE_ATTR(state, 0444, sm5507_state_show, NULL);

#if defined(CONFIG_SEC_FACTORY)
static ssize_t sm5507_rid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);

	if(!sm5507_data) {
		pr_err("%s usbpd_data is null!!\n", __func__);
		return -ENODEV;
	}
	
	return sprintf(buf, "%d\n", sm5507_data->cur_rid);

}
static DEVICE_ATTR(rid, 0444, sm5507_rid_show, NULL);

static ssize_t sm5507_ccic_control_option_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);
	int cmd;

	if(!sm5507_data) {
		pr_err("%s usbpd_data is null!!\n", __func__);
		return -ENODEV;
	}

	sscanf(buf, "%d", &cmd);
	pr_info("usb: %s mode=%d\n", __func__, cmd);

	sm5507_control_option_command(sm5507_data, cmd);

    return size;
}
static DEVICE_ATTR(ccic_control_option, 0220, NULL, sm5507_ccic_control_option_store);
#endif

static ssize_t sm5507_fw_update_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);
    u8 status;

	if(!sm5507_data) {
		pr_err("%s usbpd_data is null!!\n", __func__);
		return -ENODEV;
	}

    status = sm5507_i2c_read_byte(sm5507_data->i2c, SM5507_REG_STATUS3);

    if(status & INT_MTP_BOOT_FAIL_MASK)
    {
        pr_err("%s flash mode: %s\n", __func__, "MTP BOOT FAIL");
        sprintf(buf, "%s\n", "MTP BOOT FAIL");
    } else if (status & INT_MTP_MAIN_FAIL_MASK) {
        pr_err("%s flash mode: %s\n", __func__, "MTP MAIN FAIL");
        sprintf(buf, "%s\n", "MTP MAIN FAIL");        
    } else {
        pr_err("%s flash mode: %s\n", __func__, "MTP STATUS NORMAL");    
        sprintf(buf, "%s\n", "MTP STATUS NORMAL");        
    }


    return 0;
}
static DEVICE_ATTR(fw_update_status, 0444, sm5507_fw_update_status_show, NULL);

static ssize_t sm5507_fw_update_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sm5507_data *sm5507_data = dev_get_drvdata(dev);
	struct sm5507_version version;
	int mode = 0, ret = 1;

    if (!sm5507_data) {
        pr_err("usbpd_data is NULL\n");
        return -ENODEV;
    }

	sscanf(buf, "%d", &mode);
	pr_info("%s mode=%d\n", __func__, mode);

	sm5507_get_chip_swversion(sm5507_data, &version);
	pr_err("%s CHIP SWversion %2x %2x - before\n", __func__,
	       version.main , version.boot);

	switch (mode) {
	case BUILT_IN:
		ret = sm5507_firmware_update_built_in(dev);
		break;
	case UMS:
		ret = sm5507_firmware_update_ums(dev);
		break;
	default:
		pr_err("%s: Not support command[%d]\n",
			__func__, mode);
		break;
	}
    
	sm5507_get_chip_swversion(sm5507_data, &version);
	pr_err("%s CHIP SWversion %2x %2x - after\n", __func__,
	       version.main , version.boot);

    return size;
}
static DEVICE_ATTR(fw_update, 0220, NULL, sm5507_fw_update_store);

static struct attribute *ccic_attributes[] = {
	&dev_attr_cur_version.attr,
	&dev_attr_src_version.attr,
	&dev_attr_lpm_mode.attr,
	&dev_attr_state.attr,
#if defined(CONFIG_SEC_FACTORY)
	&dev_attr_rid.attr,
	&dev_attr_ccic_control_option.attr,
#endif
	&dev_attr_fw_update.attr,
	&dev_attr_fw_update_status.attr,
	NULL
};

const struct attribute_group ccic_sysfs_group = {
	.attrs = ccic_attributes,
};



