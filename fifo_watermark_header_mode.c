/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include "bmi2.h"
#include "bmi270.h"

/******************************************************************************/
/*!                  Macros                                                   */

/*! Buffer size allocated to store raw FIFO data */
#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(1200)

/*! Length of data to be read from FIFO */
#define BMI2_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(1200)

/*! Number of accel frames to be extracted from FIFO */

/*!
 * Calculation:
 * fifo_watermark_level = 650, accel_frame_len = 6, gyro_frame_len = 6 header_byte = 1.
 * fifo_accel_frame_count = (650 / (6 + 6 + 1 )) = 50 frames
 * NOTE: Extra frames are read in order to get sensor time
 */
#define BMI2_FIFO_ACCEL_FRAME_COUNT     UINT8_C(70)

/*! Number of gyro frames to be extracted from FIFO */
#define BMI2_FIFO_GYRO_FRAME_COUNT      UINT8_C(70)

/*! Setting a watermark level in FIFO */
#define BMI2_FIFO_WATERMARK_LEVEL       UINT16_C(650)

/*! Macro to read sensortime byte in FIFO */
#define SENSORTIME_OVERHEAD_BYTE        UINT8_C(250)

/******************************************************************************/
/*!                        Global Variables                                   */
/* To read sensortime, extra 3 bytes are added to fifo buffer */
uint16_t fifo_buffer_size = BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE;

/* Number of bytes of FIFO data
 * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
 * Array size same as fifo_buffer_size
 */
uint8_t fifo_data[BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE];

/* Array of accelerometer frames -> Total bytes =
 * 55 * (6 axes + 1 header bytes) = 385 bytes */
struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

/* Array of gyro frames -> Total bytes =
 * 55 * (6 axes + 1 header bytes) = 385 bytes */
struct bmi2_sens_axes_data fifo_gyro_data[BMI2_FIFO_GYRO_FRAME_COUNT] = { { 0 } };

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel and gyro.
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *dev);

int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    int fd = *(int *)intf_ptr;
    if (write(fd, &reg_addr, 1) != 1) return BMI2_E_COM_FAIL;
    if (read(fd, data, len) != (int)len) return BMI2_E_COM_FAIL;
    return BMI2_OK;
}

int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    int fd = *(int *)intf_ptr;
    uint8_t buf[128];
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);
    if (write(fd, buf, len + 1) != (int)(len + 1)) return BMI2_E_COM_FAIL;
    return BMI2_OK;
}

void bmi2_delay_us(uint32_t period, void* intf_ptr) {
    usleep(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:
            /* Do nothing */
            break;

        case BMI2_W_FIFO_EMPTY:
            printf("Warning [%d] : FIFO empty\r\n", rslt);
            break;
        case BMI2_W_PARTIAL_READ:
            printf("Warning [%d] : FIFO partial read\r\n", rslt);
            break;
        case BMI2_E_NULL_PTR:
            printf(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI2_E_COM_FAIL:
            printf(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI2_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_SENSOR:
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_FAIL:
            printf(
                "Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INT_PIN:
            printf(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI2_E_OUT_OF_RANGE:
            printf(
                "Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
                rslt);
            break;

        case BMI2_E_ACC_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
                rslt);
            break;

        case BMI2_E_GYRO_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
                rslt);
            break;

        case BMI2_E_ACC_GYR_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
                rslt);
            break;

        case BMI2_E_CONFIG_LOAD:
            printf(
                "Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_PAGE:
            printf(
                "Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
                rslt);
            break;

        case BMI2_E_SET_APS_FAIL:
            printf(
                "Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
                rslt);
            break;

        case BMI2_E_AUX_INVALID_CFG:
            printf(
                "Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
                rslt);
            break;

        case BMI2_E_AUX_BUSY:
            printf(
                "Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
                rslt);
            break;

        case BMI2_E_REMAP_ERROR:
            printf(
                "Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
                rslt);
            break;

        case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
            printf(
                "Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_NOT_DONE:
            printf(
                "Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INPUT:
            printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI2_E_INVALID_STATUS:
            printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI2_E_CRT_ERROR:
            printf("Error [%d] : CRT error. It occurs when the CRT test has failed\r\n", rslt);
            break;

        case BMI2_E_ST_ALREADY_RUNNING:
            printf(
                "Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
            printf(
                "Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
                rslt);
            break;

        case BMI2_E_DL_ERROR:
            printf(
                "Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
                rslt);
            break;

        case BMI2_E_PRECON_ERROR:
            printf(
                "Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_ABORT_ERROR:
            printf("Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n", rslt);
            break;

        case BMI2_E_WRITE_CYCLE_ONGOING:
            printf(
                "Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_ST_NOT_RUNING:
            printf(
                "Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
                rslt);
            break;

        case BMI2_E_DATA_RDY_INT_FAILED:
            printf(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_FOC_POSITION:
            printf(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to index bytes. */
    uint16_t index = 0;

    uint16_t fifo_length = 0;

    uint16_t accel_frame_length;

    uint16_t gyro_frame_length;

    /* Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

    /* Initialize FIFO frame structure. */
    struct bmi2_fifo_frame fifoframe = { 0 };

    /* Accel and gyro sensor are listed in array. */
    uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_GYRO };

    /* Variable to get fifo water-mark interrupt status. */
    uint16_t int_status = 0;

    uint16_t watermark = 0;

    int i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0 || ioctl(i2c_fd, I2C_SLAVE, 0x68) < 0) {
        printf("can not open i2c\n");
        return 0;
    } else printf("i2c ok\n");

    bmi2_dev.intf = BMI2_I2C_INTF;
    bmi2_dev.intf_ptr = &i2c_fd;
    bmi2_dev.read = bmi2_i2c_read;
    bmi2_dev.write = bmi2_i2c_write;
    bmi2_dev.delay_us = bmi2_delay_us;
    bmi2_dev.read_write_len = 32;
    bmi2_dev.config_file_ptr = NULL;

    /* Initialize bmi270. */
    rslt = bmi270_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Configuration settings for accel and gyro. */
    rslt = set_accel_gyro_config(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* NOTE:
     * Accel and Gyro enable must be done after setting configurations
     */
    rslt = bmi270_sensor_enable(sensor_sel, 2, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Before setting FIFO, disable the advance power save mode. */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Initially disable all configurations in fifo. */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Update FIFO structure. */
    /* Mapping the buffer to store the fifo data. */
    fifoframe.data = fifo_data;

    /* Length of FIFO frame. */
    /* To read sensortime, extra 3 bytes are added to fifo user length. */
    fifoframe.length = BMI2_FIFO_RAW_DATA_USER_LENGTH + SENSORTIME_OVERHEAD_BYTE;

    /* Set FIFO configuration by enabling accel, gyro and timestamp.
     * NOTE 1: The header mode is enabled by default.
     * NOTE 2: By default the FIFO operating mode is FIFO mode.
     * NOTE 3: Sensortime is enabled by default */
    printf("FIFO is configured in header mode\n");
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN, BMI2_ENABLE, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* FIFO water-mark interrupt is enabled. */
    fifoframe.data_int_map = BMI2_FWM_INT;

    /* Map water-mark interrupt to the required interrupt pin. */
    rslt = bmi2_map_data_int(fifoframe.data_int_map, BMI2_INT1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Set water-mark level. */
    fifoframe.wm_lvl = BMI2_FIFO_WATERMARK_LEVEL;

    /* Set the water-mark level if water-mark interrupt is mapped. */
    rslt = bmi2_set_fifo_wm(fifoframe.wm_lvl, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    float acc_scale = (4.0f * 9.80665f) / 32768.0f; // Scale factor for +/-4G range (m/s^2 per LSB)
    float gyro_scale = (2000.0f / 32768.0f) * (M_PI / 180.0f); // Scale factor for +/-2000 dps range (rad/s per LSB)

    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    int64_t local_ns = tp.tv_sec * 1000000000 + tp.tv_nsec;
    uint8_t sensortime_raw[3] = { 0 };
    rslt = bmi2_get_regs(BMI2_CHIP_ID_ADDR, sensortime_raw, 3, &bmi2_dev);
    int64_t time_offset_ns = 0;
    if (rslt == BMI2_OK) {
        uint32_t sensortime = 0;
        sensortime = (uint32_t)sensortime_raw[0] | ((uint32_t)sensortime_raw[1] << 8) | ((uint32_t)sensortime_raw[2] << 16);
        //printf("sensor time (s) %f\n", sensortime * BMI2_SENSORTIME_RESOLUTION);
        int64_t sensortime_ns = sensortime * 39062.5;
        printf("local_ns / sensortime_ns: %ld / %ld\n", local_ns, sensortime_ns);
        time_offset_ns = local_ns - sensortime_ns;
    }

    while (1)
    {
        /* Read FIFO data on interrupt. */
        rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* To check the status of FIFO watermark interrupt. */
        if ((rslt == BMI2_OK) && (int_status & BMI2_FWM_INT_STATUS_MASK))
        {
            rslt = bmi2_get_fifo_wm(&watermark, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);
            printf("\nFIFO watermark level : %d\n", watermark);

            rslt = bmi2_get_fifo_length(&fifo_length, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            printf("\nFIFO data bytes available : %d \n", fifo_length);
            printf("\nFIFO data bytes requested : %d \n", fifoframe.length);

            /* Updating FIFO length to be read based on available length and dummy byte updation */
            fifoframe.length = fifo_length + SENSORTIME_OVERHEAD_BYTE + bmi2_dev.dummy_byte;

            /* Read FIFO data. */
            rslt = bmi2_read_fifo_data(&fifoframe, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            /* Read FIFO data on interrupt. */
            rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;
                gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

                printf("\nFIFO accel frames requested : %d \n", accel_frame_length);

                /* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
                (void)bmi2_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, &bmi2_dev);
                printf("\nFIFO accel frames extracted : %d \n", accel_frame_length);

                printf("\nFIFO gyro frames requested : %d \n", gyro_frame_length);

                /* Parse the FIFO data to extract gyro data from the FIFO buffer. */
                (void)bmi2_extract_gyro(fifo_gyro_data, &gyro_frame_length, &fifoframe, &bmi2_dev);
                printf("\nFIFO gyro frames extracted : %d \n", gyro_frame_length);

                printf("\nExtracted accel frames\n");

                printf("ACCEL_DATA, X, Y, Z\n");

                /* Print the parsed accelerometer data from the FIFO buffer. */
                for (index = 0; index < accel_frame_length; index++)
                {
                    printf("%d, %f, %f, %f\n",
                           index,
                           fifo_accel_data[index].x * acc_scale,
                           fifo_accel_data[index].y * acc_scale,
                           fifo_accel_data[index].z * acc_scale);
                }

                printf("\nExtracted gyro frames\n");

                printf("GYRO_DATA, X, Y, Z\n");

                /* Print the parsed gyro data from the FIFO buffer. */
                for (index = 0; index < gyro_frame_length; index++)
                {
                    printf("%d, %f, %f, %f\n", index, fifo_gyro_data[index].x * gyro_scale, fifo_gyro_data[index].y * gyro_scale,
                           fifo_gyro_data[index].z * gyro_scale);
                }

                /* Print control frames like sensor time and skipped frame count. */
                printf("Skipped frame count = %d\n", fifoframe.skipped_frame_count);
                printf("Sensor time(in seconds) = %.4lf  s\r\n", (fifoframe.sensor_time * BMI2_SENSORTIME_RESOLUTION));
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accel and gyro configurations. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(config, 2, dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Accel configuration settings. */
        /* Set Output Data Rate */
        config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Gyro configuration settings. */
        /* Set Output Data Rate */
        config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope Bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set new configurations. */
        rslt = bmi270_set_sensor_config(config, 2, dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}
