/****************************************************************************
 ****************************************************************************
 ***
 ***   This header was automatically generated from a Linux kernel header
 ***   of the same name, to make information necessary for userspace to
 ***   call into the kernel available to libc.  It contains only constants,
 ***   structures, and macros generated from the original header, and thus,
 ***   contains no copyrightable information.
 ***
 ***   To edit the content of this header, modify the corresponding
 ***   source file (e.g. under external/kernel-headers/original/) then
 ***   run bionic/libc/kernel/tools/update_all.py
 ***
 ***   Any manual change here will be lost the next time this script will
 ***   be run. You've been warned!
 ***
 ****************************************************************************
 ****************************************************************************/
#ifndef __LINUX_MSM_CAM_SENSOR_H
#define __LINUX_MSM_CAM_SENSOR_H
#ifdef MSM_CAMERA_BIONIC
#include <sys/types.h>
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#endif
#include <linux/types.h>
#include <linux/v4l2-mediabus.h>
#include <linux/i2c.h>
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define I2C_SEQ_REG_SETTING_MAX 5
#define I2C_SEQ_REG_DATA_MAX 20
#define MAX_CID 16
#define MSM_SENSOR_MCLK_8HZ 8000000
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define MSM_SENSOR_MCLK_16HZ 16000000
#define MSM_SENSOR_MCLK_24HZ 24000000
#define GPIO_OUT_LOW (0 << 1)
#define GPIO_OUT_HIGH (1 << 1)
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define CSI_EMBED_DATA 0x12
#define CSI_RESERVED_DATA_0 0x13
#define CSI_YUV422_8 0x1E
#define CSI_RAW8 0x2A
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define CSI_RAW10 0x2B
#define CSI_RAW12 0x2C
#define CSI_DECODE_6BIT 0
#define CSI_DECODE_8BIT 1
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define CSI_DECODE_10BIT 2
#define CSI_DECODE_DPCM_10_8_10 5
#define MAX_SENSOR_NAME 32
#define MAX_ACT_MOD_NAME_SIZE 32
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define MAX_ACT_NAME_SIZE 32
#define NUM_ACTUATOR_DIR 2
#define MAX_ACTUATOR_SCENARIO 8
#define MAX_ACTUATOR_REGION 5
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define MAX_ACTUATOR_INIT_SET 12
#define MAX_ACTUATOR_REG_TBL_SIZE 8
#define MAX_ACTUATOR_AF_TOTAL_STEPS 1024
#define MAX_OIS_MOD_NAME_SIZE 32
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define MAX_OIS_NAME_SIZE 32
#define MAX_OIS_REG_SETTINGS 800
#define MOVE_NEAR 0
#define MOVE_FAR 1
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define MSM_ACTUATOR_MOVE_SIGNED_FAR -1
#define MSM_ACTUATOR_MOVE_SIGNED_NEAR 1
#define MAX_EEPROM_NAME 32
#define MAX_AF_ITERATIONS 3
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define MAX_NUMBER_OF_STEPS 47
#define MAX_LED_TRIGGERS 3
enum sensor_stats_type {
 YRGB,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 YYYY,
};
enum flash_type {
 LED_FLASH = 1,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 STROBE_FLASH,
 GPIO_FLASH
};
enum msm_camera_i2c_reg_addr_type {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_I2C_BYTE_ADDR = 1,
 MSM_CAMERA_I2C_WORD_ADDR,
 MSM_CAMERA_I2C_3B_ADDR,
 MSM_CAMERA_I2C_ADDR_TYPE_MAX,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
enum msm_camera_i2c_data_type {
 MSM_CAMERA_I2C_BYTE_DATA = 1,
 MSM_CAMERA_I2C_WORD_DATA,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_I2C_DWORD_DATA,
 MSM_CAMERA_I2C_SET_BYTE_MASK,
 MSM_CAMERA_I2C_UNSET_BYTE_MASK,
 MSM_CAMERA_I2C_SET_WORD_MASK,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_I2C_UNSET_WORD_MASK,
 MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA,
 MSM_CAMERA_I2C_DATA_TYPE_MAX,
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
enum msm_sensor_power_seq_type_t {
 SENSOR_CLK,
 SENSOR_GPIO,
 SENSOR_VREG,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SENSOR_I2C_MUX,
};
enum msm_sensor_clk_type_t {
 SENSOR_CAM_MCLK,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SENSOR_CAM_CLK,
 SENSOR_CAM_CLK_MAX,
};
enum msm_sensor_power_seq_gpio_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SENSOR_GPIO_RESET,
 SENSOR_GPIO_STANDBY,
 SENSOR_GPIO_AF_PWDM,
 SENSOR_GPIO_VIO,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SENSOR_GPIO_VANA,
 SENSOR_GPIO_VDIG,
 SENSOR_GPIO_VAF,
 SENSOR_GPIO_FL_EN,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SENSOR_GPIO_FL_NOW,
 SENSOR_GPIO_TOR_EN,
 SENSOR_GPIO_MAX,
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
enum msm_camera_vreg_name_t {
 CAM_VDIG,
 CAM_VIO,
 CAM_VANA,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CAM_VAF,
 CAM_VREG_MAX,
};
enum msm_sensor_resolution_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_SENSOR_RES_FULL,
 MSM_SENSOR_RES_QTR,
 MSM_SENSOR_RES_2,
 MSM_SENSOR_RES_3,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_SENSOR_RES_4,
 MSM_SENSOR_RES_5,
 MSM_SENSOR_RES_6,
 MSM_SENSOR_RES_7,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_SENSOR_INVALID_RES,
};
enum msm_camera_stream_type_t {
 MSM_CAMERA_STREAM_PREVIEW,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_STREAM_SNAPSHOT,
 MSM_CAMERA_STREAM_VIDEO,
 MSM_CAMERA_STREAM_INVALID,
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
enum sensor_sub_module_t {
 SUB_MODULE_SENSOR,
 SUB_MODULE_CHROMATIX,
 SUB_MODULE_ACTUATOR,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SUB_MODULE_EEPROM,
 SUB_MODULE_LED_FLASH,
 SUB_MODULE_STROBE_FLASH,
 SUB_MODULE_CSID,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SUB_MODULE_CSID_3D,
 SUB_MODULE_CSIPHY,
 SUB_MODULE_CSIPHY_3D,
 SUB_MODULE_OIS,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SUB_MODULE_MAX,
};
struct otp_info_t {
 uint8_t enable;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t page_size;
 uint16_t num_of_pages;
 uint16_t page_reg_addr;
 uint16_t page_reg_base_addr;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t ctrl_reg_addr;
 uint16_t ctrl_reg_read_mode;
 uint16_t status_reg_addr;
 uint16_t status_reg_read_complete_bit;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t reset_reg_addr;
 uint16_t reset_reg_stream_on;
 uint16_t reset_reg_stream_off;
 uint16_t data_seg_addr;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum msm_camera_i2c_data_type data_size;
 uint8_t big_endian;
 uint8_t poll_times;
 uint16_t poll_usleep;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t *otp_info;
 uint8_t otp_read;
};
enum {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_EFFECT_MODE_OFF,
 MSM_CAMERA_EFFECT_MODE_MONO,
 MSM_CAMERA_EFFECT_MODE_NEGATIVE,
 MSM_CAMERA_EFFECT_MODE_SOLARIZE,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_EFFECT_MODE_SEPIA,
 MSM_CAMERA_EFFECT_MODE_POSTERIZE,
 MSM_CAMERA_EFFECT_MODE_WHITEBOARD,
 MSM_CAMERA_EFFECT_MODE_BLACKBOARD,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_EFFECT_MODE_AQUA,
 MSM_CAMERA_EFFECT_MODE_EMBOSS,
 MSM_CAMERA_EFFECT_MODE_SKETCH,
 MSM_CAMERA_EFFECT_MODE_NEON,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_EFFECT_MODE_MAX
};
enum {
 MSM_CAMERA_WB_MODE_AUTO,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_WB_MODE_CUSTOM,
 MSM_CAMERA_WB_MODE_INCANDESCENT,
 MSM_CAMERA_WB_MODE_FLUORESCENT,
 MSM_CAMERA_WB_MODE_WARM_FLUORESCENT,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_WB_MODE_DAYLIGHT,
 MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT,
 MSM_CAMERA_WB_MODE_TWILIGHT,
 MSM_CAMERA_WB_MODE_SHADE,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_WB_MODE_OFF,
 MSM_CAMERA_WB_MODE_MAX
};
enum {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_SCENE_MODE_OFF,
 MSM_CAMERA_SCENE_MODE_AUTO,
 MSM_CAMERA_SCENE_MODE_LANDSCAPE,
 MSM_CAMERA_SCENE_MODE_SNOW,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_SCENE_MODE_BEACH,
 MSM_CAMERA_SCENE_MODE_SUNSET,
 MSM_CAMERA_SCENE_MODE_NIGHT,
 MSM_CAMERA_SCENE_MODE_PORTRAIT,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_SCENE_MODE_BACKLIGHT,
 MSM_CAMERA_SCENE_MODE_SPORTS,
 MSM_CAMERA_SCENE_MODE_ANTISHAKE,
 MSM_CAMERA_SCENE_MODE_FLOWERS,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_SCENE_MODE_CANDLELIGHT,
 MSM_CAMERA_SCENE_MODE_FIREWORKS,
 MSM_CAMERA_SCENE_MODE_PARTY,
 MSM_CAMERA_SCENE_MODE_NIGHT_PORTRAIT,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_SCENE_MODE_THEATRE,
 MSM_CAMERA_SCENE_MODE_ACTION,
 MSM_CAMERA_SCENE_MODE_AR,
 MSM_CAMERA_SCENE_MODE_FACE_PRIORITY,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_SCENE_MODE_BARCODE,
 MSM_CAMERA_SCENE_MODE_HDR,
 MSM_CAMERA_SCENE_MODE_MAX
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
enum csid_cfg_type_t {
 CSID_INIT,
 CSID_CFG,
 CSID_RELEASE,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
enum csiphy_cfg_type_t {
 CSIPHY_INIT,
 CSIPHY_CFG,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CSIPHY_RELEASE,
};
enum camera_vreg_type {
 REG_LDO,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 REG_VS,
 REG_GPIO,
};
enum sensor_af_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 SENSOR_AF_FOCUSSED,
 SENSOR_AF_NOT_FOCUSSED,
};
struct msm_sensor_power_setting {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum msm_sensor_power_seq_type_t seq_type;
 uint16_t seq_val;
 long config_val;
 uint16_t delay;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 void *data[10];
};
struct msm_sensor_power_setting_array {
 struct msm_sensor_power_setting *power_setting;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t size;
 struct msm_sensor_power_setting *power_down_setting;
 uint16_t size_down;
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_sensor_id_info_t {
 uint16_t sensor_id_reg_addr;
 uint16_t sensor_id;
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
enum msm_sensor_camera_id_t {
 CAMERA_0,
 CAMERA_1,
 CAMERA_2,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CAMERA_3,
 MAX_CAMERAS,
};
enum cci_i2c_master_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MASTER_0,
 MASTER_1,
 MASTER_MAX,
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
enum i2c_freq_mode_t {
 I2C_STANDARD_MODE,
 I2C_FAST_MODE,
 I2C_CUSTOM_MODE,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 I2C_MAX_MODES,
};
struct msm_camera_i2c_reg_array {
 uint16_t reg_addr;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t reg_data;
 uint32_t delay;
};
struct msm_camera_i2c_reg_setting {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct msm_camera_i2c_reg_array *reg_setting;
 uint16_t size;
 enum msm_camera_i2c_reg_addr_type addr_type;
 enum msm_camera_i2c_data_type data_type;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t delay;
};
struct msm_camera_i2c_seq_reg_array {
 uint16_t reg_addr;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t reg_data[I2C_SEQ_REG_DATA_MAX];
 uint16_t reg_data_size;
};
struct msm_camera_i2c_seq_reg_setting {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct msm_camera_i2c_seq_reg_array *reg_setting;
 uint16_t size;
 enum msm_camera_i2c_reg_addr_type addr_type;
 uint16_t delay;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_camera_i2c_array_write_config {
 struct msm_camera_i2c_reg_setting conf_array;
 uint16_t slave_addr;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_camera_i2c_read_config {
 uint16_t slave_addr;
 uint16_t reg_addr;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum msm_camera_i2c_data_type data_type;
 uint16_t *data;
};
struct msm_camera_csid_vc_cfg {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t cid;
 uint8_t dt;
 uint8_t decode_format;
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_camera_csid_lut_params {
 uint8_t num_cid;
 struct msm_camera_csid_vc_cfg *vc_cfg[MAX_CID];
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_camera_csid_params {
 uint8_t lane_cnt;
 uint16_t lane_assign;
 uint8_t phy_sel;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct msm_camera_csid_lut_params lut_params;
};
struct msm_camera_csiphy_params {
 uint8_t lane_cnt;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t settle_cnt;
 uint16_t lane_mask;
 uint8_t combo_mode;
 uint8_t csid_core;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_camera_csi2_params {
 struct msm_camera_csid_params csid_params;
 struct msm_camera_csiphy_params csiphy_params;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_camera_csi_lane_params {
 uint16_t csi_lane_assign;
 uint16_t csi_lane_mask;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct csi_lane_params_t {
 uint16_t csi_lane_assign;
 uint8_t csi_lane_mask;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t csi_if;
 uint8_t csid_core[2];
 uint8_t csi_phy_sel;
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
enum camb_position_t {
 BACK_CAMERA_B,
 FRONT_CAMERA_B,
 INVALID_CAMERA_B,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_sensor_info_t {
 char sensor_name[MAX_SENSOR_NAME];
 uint32_t session_id;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 int32_t subdev_id[SUB_MODULE_MAX];
 uint8_t is_mount_angle_valid;
 uint32_t sensor_mount_angle;
 int modes_supported;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum camb_position_t position;
};
struct camera_vreg_t {
 const char *reg_name;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 int min_voltage;
 int max_voltage;
 int op_mode;
 uint32_t delay;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
enum camerab_mode_t {
 CAMERA_MODE_2D_B = (1<<0),
 CAMERA_MODE_3D_B = (1<<1),
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CAMERA_MODE_INVALID = (1<<2),
};
struct msm_sensor_init_params {
 int modes_supported;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum camb_position_t position;
 uint32_t sensor_mount_angle;
 struct otp_info_t sensor_otp;
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_camera_sensor_slave_info {
 char sensor_name[32];
 char eeprom_name[32];
 char actuator_name[32];
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 char ois_name[32];
 enum msm_sensor_camera_id_t camera_id;
 uint16_t slave_addr;
 enum i2c_freq_mode_t i2c_freq_mode;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum msm_camera_i2c_reg_addr_type addr_type;
 struct msm_sensor_id_info_t sensor_id_info;
 struct msm_sensor_power_setting_array power_setting_array;
 uint8_t is_init_params_valid;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct msm_sensor_init_params sensor_init_params;
 uint8_t is_flash_supported;
};
struct sensorb_cfg_data {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 int cfgtype;
 union {
 struct msm_sensor_info_t sensor_info;
 struct msm_sensor_init_params sensor_init_params;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 void *setting;
 } cfg;
};
struct csid_cfg_data {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum csid_cfg_type_t cfgtype;
 union {
 uint32_t csid_version;
 struct msm_camera_csid_params *csid_params;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 } cfg;
};
struct csiphy_cfg_data {
 enum csiphy_cfg_type_t cfgtype;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 union {
 struct msm_camera_csiphy_params *csiphy_params;
 struct msm_camera_csi_lane_params *csi_lane_params;
 } cfg;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
enum eeprom_cfg_type_t {
 CFG_EEPROM_GET_INFO,
 CFG_EEPROM_GET_CAL_DATA,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_EEPROM_READ_CAL_DATA,
 CFG_EEPROM_WRITE_DATA,
 CFG_EEPROM_GET_MM_INFO,
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct eeprom_get_t {
 uint32_t num_bytes;
};
struct eeprom_read_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t *dbuffer;
 uint32_t num_bytes;
};
struct eeprom_write_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t *dbuffer;
 uint32_t num_bytes;
};
struct eeprom_get_cmm_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t cmm_support;
 uint32_t cmm_compression;
 uint32_t cmm_size;
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_eeprom_cfg_data {
 enum eeprom_cfg_type_t cfgtype;
 uint8_t is_supported;
 union {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 char eeprom_name[MAX_SENSOR_NAME];
 struct eeprom_get_t get_data;
 struct eeprom_read_t read_data;
 struct eeprom_write_t write_data;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct eeprom_get_cmm_t get_cmm_data;
 } cfg;
};
enum msm_sensor_cfg_type_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_SET_SLAVE_INFO,
 CFG_SLAVE_READ_I2C,
 CFG_WRITE_I2C_ARRAY,
 CFG_SLAVE_WRITE_I2C_ARRAY,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_WRITE_I2C_SEQ_ARRAY,
 CFG_POWER_UP,
 CFG_POWER_DOWN,
 CFG_SET_STOP_STREAM_SETTING,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_GET_SENSOR_INFO,
 CFG_GET_MODULE_INFO,
 CFG_GET_SENSOR_INIT_PARAMS,
 CFG_SET_INIT_SETTING,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_SET_RESOLUTION,
 CFG_SET_STOP_STREAM,
 CFG_SET_START_STREAM,
 CFG_SET_SATURATION,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_SET_CONTRAST,
 CFG_SET_SHARPNESS,
 CFG_SET_ISO,
 CFG_SET_EXPOSURE_COMPENSATION,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_SET_ANTIBANDING,
 CFG_SET_BESTSHOT_MODE,
 CFG_SET_EFFECT,
 CFG_SET_WHITE_BALANCE,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_SET_AUTOFOCUS,
 CFG_CANCEL_AUTOFOCUS,
 CFG_SET_STREAM_TYPE,
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
enum msm_actuator_cfg_type_t {
 CFG_GET_ACTUATOR_INFO,
 CFG_SET_ACTUATOR_INFO,
 CFG_SET_DEFAULT_FOCUS,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_SET_POSITION,
 CFG_MOVE_FOCUS,
 CFG_ACTUATOR_POWERDOWN,
 CFG_ACTUATOR_POWERUP,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_ACTUATOR_INIT,
 CFG_DIRECT_I2C_WRITE,
};
enum msm_ois_cfg_type_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_OIS_INIT,
 CFG_GET_OIS_INFO,
 CFG_OIS_POWERDOWN,
 CFG_OIS_INI_SET,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_OIS_ENABLE,
 CFG_OIS_DISABLE,
 CFG_OIS_SET_MOVIE_MODE,
 CFG_OIS_SET_STILL_MODE,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_OIS_SET_CENTERING_ON,
 CFG_OIS_SET_PANTILT_ON,
 CFG_OIS_POWERUP,
 CFG_OIS_I2C_WRITE_SEQ_TABLE,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
enum msm_ois_i2c_operation {
 MSM_OIS_WRITE = 0,
 MSM_OIS_POLL,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct reg_settings_ois_t {
 uint16_t reg_addr;
 enum msm_camera_i2c_reg_addr_type addr_type;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t reg_data;
 enum msm_camera_i2c_data_type data_type;
 enum msm_ois_i2c_operation i2c_operation;
 uint32_t delay;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_ois_params_t {
 uint16_t data_size;
 uint16_t init_setting_size;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t enable_ois_setting_size;
 uint16_t disable_ois_setting_size;
 uint16_t movie_mode_ois_setting_size;
 uint16_t still_mode_ois_setting_size;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t centering_on_ois_setting_size;
 uint16_t centering_off_ois_setting_size;
 uint16_t pantilt_on_ois_setting_size;
 uint32_t i2c_addr;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum msm_camera_i2c_reg_addr_type i2c_addr_type;
 enum msm_camera_i2c_data_type i2c_data_type;
 struct reg_settings_ois_t *init_settings;
 struct reg_settings_ois_t *enable_ois_settings;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct reg_settings_ois_t *disable_ois_settings;
 struct reg_settings_ois_t *movie_mode_ois_settings;
 struct reg_settings_ois_t *still_mode_ois_settings;
 struct reg_settings_ois_t *centering_on_ois_settings;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct reg_settings_ois_t *centering_off_ois_settings;
 struct reg_settings_ois_t *pantilt_on_ois_settings;
};
struct msm_ois_set_info_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct msm_ois_params_t ois_params;
};
enum actuator_type {
 ACTUATOR_VCM,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 ACTUATOR_PIEZO,
 ACTUATOR_HVCM,
};
enum msm_actuator_data_type {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_ACTUATOR_BYTE_DATA = 1,
 MSM_ACTUATOR_WORD_DATA,
};
enum msm_actuator_addr_type {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_ACTUATOR_BYTE_ADDR = 1,
 MSM_ACTUATOR_WORD_ADDR,
};
enum msm_actuator_i2c_operation {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_ACT_WRITE = 0,
 MSM_ACT_POLL,
};
struct reg_settings_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t reg_addr;
 enum msm_actuator_addr_type addr_type;
 uint16_t reg_data;
 enum msm_actuator_data_type data_type;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum msm_actuator_i2c_operation i2c_operation;
 uint32_t delay;
};
struct region_params_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t step_bound[2];
 uint16_t code_per_step;
};
struct damping_params_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t damping_step;
 uint32_t damping_delay;
 uint32_t hw_params;
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_actuator_move_params_t {
 int8_t dir;
 int8_t sign_dir;
 int16_t dest_step_pos;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 int32_t num_steps;
 uint16_t curr_lens_pos;
 struct damping_params_t *ringing_params;
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_actuator_tuning_params_t {
 int16_t initial_code;
 uint16_t pwd_step;
 uint16_t region_size;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t total_steps;
 struct region_params_t *region_params;
};
struct park_lens_data_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t damping_step;
 uint32_t damping_delay;
 uint32_t hw_params;
 uint32_t max_step;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_actuator_params_t {
 enum actuator_type act_type;
 uint8_t reg_tbl_size;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t data_size;
 uint16_t init_setting_size;
 uint32_t i2c_addr;
 enum msm_actuator_addr_type i2c_addr_type;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum msm_actuator_data_type i2c_data_type;
 struct msm_actuator_reg_params_t *reg_tbl_params;
 struct reg_settings_t *init_settings;
 struct park_lens_data_t park_lens;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_actuator_set_info_t {
 struct msm_actuator_params_t actuator_params;
 struct msm_actuator_tuning_params_t af_tuning_params;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
struct msm_actuator_get_info_t {
 uint32_t focal_length_num;
 uint32_t focal_length_den;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t f_number_num;
 uint32_t f_number_den;
 uint32_t f_pix_num;
 uint32_t f_pix_den;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t total_f_dist_num;
 uint32_t total_f_dist_den;
 uint32_t hor_view_angle_num;
 uint32_t hor_view_angle_den;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t ver_view_angle_num;
 uint32_t ver_view_angle_den;
};
enum af_camera_name {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 ACTUATOR_MAIN_CAM_0,
 ACTUATOR_MAIN_CAM_1,
 ACTUATOR_MAIN_CAM_2,
 ACTUATOR_MAIN_CAM_3,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 ACTUATOR_MAIN_CAM_4,
 ACTUATOR_MAIN_CAM_5,
 ACTUATOR_WEB_CAM_0,
 ACTUATOR_WEB_CAM_1,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 ACTUATOR_WEB_CAM_2,
};
struct msm_ois_cfg_data {
 int cfgtype;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t is_ois_supported;
 union {
 uint8_t enable_centering_ois;
 struct msm_ois_set_info_t set_info;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct msm_camera_i2c_seq_reg_setting *settings;
 } cfg;
};
struct msm_actuator_set_position_t {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint16_t number_of_steps;
 uint16_t pos[MAX_NUMBER_OF_STEPS];
 uint16_t delay[MAX_NUMBER_OF_STEPS];
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_actuator_i2c {
 uint16_t addr;
 uint16_t value;
 uint32_t wait_time;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
#define MSM_ACTUATOR_I2C_MAX_TABLE_SIZE (8)
struct msm_actuator_i2c_table {
 struct msm_actuator_i2c data[MSM_ACTUATOR_I2C_MAX_TABLE_SIZE];
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t size;
};
struct msm_actuator_cfg_data {
 int cfgtype;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint8_t is_af_supported;
 union {
 struct msm_actuator_move_params_t move;
 struct msm_actuator_set_info_t set_info;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 struct msm_actuator_get_info_t get_info;
 struct msm_actuator_set_position_t setpos;
 enum af_camera_name cam_name;
 struct msm_actuator_i2c_table i2c_table;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 } cfg;
};
enum msm_actuator_write_type {
 MSM_ACTUATOR_WRITE_HW_DAMP,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_ACTUATOR_WRITE_DAC,
};
struct msm_actuator_reg_params_t {
 enum msm_actuator_write_type reg_write_type;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t hw_mask;
 uint16_t reg_addr;
 uint16_t hw_shift;
 uint16_t data_shift;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
enum msm_camera_led_config_t {
 MSM_CAMERA_LED_OFF,
 MSM_CAMERA_LED_LOW,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 MSM_CAMERA_LED_HIGH,
 MSM_CAMERA_LED_INIT,
 MSM_CAMERA_LED_RELEASE,
};
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
struct msm_camera_led_cfg_t {
 enum msm_camera_led_config_t cfgtype;
 uint32_t torch_current;
 uint32_t flash_current[MAX_LED_TRIGGERS];
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 uint32_t flash_duration[MAX_LED_TRIGGERS];
};
enum msm_sensor_init_cfg_type_t {
 CFG_SINIT_PROBE,
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 CFG_SINIT_PROBE_DONE,
 CFG_SINIT_PROBE_WAIT_DONE,
};
struct sensor_init_cfg_data {
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
 enum msm_sensor_init_cfg_type_t cfgtype;
 union {
 void *setting;
 } cfg;
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
};
#define VIDIOC_MSM_SENSOR_CFG   _IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct sensorb_cfg_data)
#define VIDIOC_MSM_SENSOR_RELEASE   _IO('V', BASE_VIDIOC_PRIVATE + 2)
#define VIDIOC_MSM_SENSOR_GET_SUBDEV_ID   _IOWR('V', BASE_VIDIOC_PRIVATE + 3, uint32_t)
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define VIDIOC_MSM_CSIPHY_IO_CFG   _IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct csiphy_cfg_data)
#define VIDIOC_MSM_CSID_IO_CFG   _IOWR('V', BASE_VIDIOC_PRIVATE + 5, struct csid_cfg_data)
#define VIDIOC_MSM_ACTUATOR_CFG   _IOWR('V', BASE_VIDIOC_PRIVATE + 6, struct msm_actuator_cfg_data)
#define VIDIOC_MSM_FLASH_LED_DATA_CFG   _IOWR('V', BASE_VIDIOC_PRIVATE + 7, struct msm_camera_led_cfg_t)
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define VIDIOC_MSM_EEPROM_CFG   _IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct msm_eeprom_cfg_data)
#define VIDIOC_MSM_SENSOR_GET_AF_STATUS   _IOWR('V', BASE_VIDIOC_PRIVATE + 9, uint32_t)
#define VIDIOC_MSM_SENSOR_INIT_CFG   _IOWR('V', BASE_VIDIOC_PRIVATE + 10, struct sensor_init_cfg_data)
#define VIDIOC_MSM_OIS_CFG   _IOWR('V', BASE_VIDIOC_PRIVATE + 11, struct msm_ois_cfg_data)
/* WARNING: DO NOT EDIT, AUTO-GENERATED CODE - SEE TOP FOR INSTRUCTIONS */
#define MSM_V4L2_PIX_FMT_META v4l2_fourcc('M', 'E', 'T', 'A')
#endif
