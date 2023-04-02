#define CONFIG_NRF5340_AUDIO 1
#define CONFIG_AUDIO_DEV 1
#define CONFIG_TRANSPORT_CIS 1
#define CONFIG_REBOOT 1
#define CONFIG_MAIN_THREAD_PRIORITY 10
#define CONFIG_MAIN_STACK_SIZE 3616
#define CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE 4096
#define CONFIG_THREAD_NAME 1
#define CONFIG_HW_CODEC_CIRRUS_LOGIC 1
#define CONFIG_BT 1
#define CONFIG_BOARD_ENABLE_DCDC_APP 1
#define CONFIG_BOARD_ENABLE_DCDC_NET 1
#define CONFIG_BOARD_ENABLE_CPUNET 1
#define CONFIG_NFCT_PINS_AS_GPIOS 1
#define CONFIG_NRFX_I2S 1
#define CONFIG_NRFX_CLOCK 1
#define CONFIG_I2C 1
#define CONFIG_FPU 1
#define CONFIG_FPU_SHARING 1
#define CONFIG_DISK_DRIVERS 1
#define CONFIG_DISK_DRIVER_SDMMC 1
#define CONFIG_SPI 1
#define CONFIG_ADC 1
#define CONFIG_SPI_NRFX_RAM_BUFFER_SIZE 8
#define CONFIG_FILE_SYSTEM 1
#define CONFIG_FAT_FILESYSTEM_ELM 1
#define CONFIG_FS_FATFS_LFN 1
#define CONFIG_FS_FATFS_EXFAT 1
#define CONFIG_FS_FATFS_MAX_LFN 255
#define CONFIG_NRFX_TIMER1 1
#define CONFIG_NRFX_DPPI 1
#define CONFIG_CMSIS_DSP 1
#define CONFIG_CMSIS_DSP_FASTMATH 1
#define CONFIG_LC3_ENC_CHAN_MAX 1
#define CONFIG_LC3_DEC_CHAN_MAX 1
#define CONFIG_AUDIO_TEST_TONE 1
#define CONFIG_AUDIO_FRAME_DURATION_10_MS 1
#define CONFIG_AUDIO_FRAME_DURATION_US 10000
#define CONFIG_AUDIO_SAMPLE_RATE_48000_HZ 1
#define CONFIG_AUDIO_SAMPLE_RATE_HZ 48000
#define CONFIG_AUDIO_BIT_DEPTH_16 1
#define CONFIG_AUDIO_BIT_DEPTH_BITS 16
#define CONFIG_AUDIO_BIT_DEPTH_OCTETS 2
#define CONFIG_AUDIO_SOURCE_I2S 1
#define CONFIG_AUDIO_HEADSET_CHANNEL_RUNTIME 1
#define CONFIG_SW_CODEC_LC3 1
#define CONFIG_LC3_BITRATE 96000
#define CONFIG_LC3_BITRATE_MAX 124000
#define CONFIG_LC3_BITRATE_MIN 32000
#define CONFIG_SW_CODEC_LC3_T2_SOFTWARE 1
#define CONFIG_LC3_ENC_SAMPLE_RATE_8KHZ_SUPPORT 1
#define CONFIG_LC3_ENC_SAMPLE_RATE_16KHZ_SUPPORT 1
#define CONFIG_LC3_ENC_SAMPLE_RATE_24KHZ_SUPPORT 1
#define CONFIG_LC3_ENC_SAMPLE_RATE_32KHZ_SUPPORT 1
#define CONFIG_LC3_ENC_SAMPLE_RATE_441KHZ_SUPPORT 1
#define CONFIG_LC3_ENC_SAMPLE_RATE_48KHZ_SUPPORT 1
#define CONFIG_LC3_DEC_SAMPLE_RATE_8KHZ_SUPPORT 1
#define CONFIG_LC3_DEC_SAMPLE_RATE_16KHZ_SUPPORT 1
#define CONFIG_LC3_DEC_SAMPLE_RATE_24KHZ_SUPPORT 1
#define CONFIG_LC3_DEC_SAMPLE_RATE_32KHZ_SUPPORT 1
#define CONFIG_LC3_DEC_SAMPLE_RATE_441KHZ_SUPPORT 1
#define CONFIG_LC3_DEC_SAMPLE_RATE_48KHZ_SUPPORT 1
#define CONFIG_BUF_BLE_RX_PACKET_NUM 5
#define CONFIG_STREAM_BIDIRECTIONAL 1
#define CONFIG_WALKIE_TALKIE_DEMO 1
#define CONFIG_AUDIO_SYSTEM_LOG_LEVEL_INF 1
#define CONFIG_AUDIO_SYSTEM_LOG_LEVEL 3
#define CONFIG_SW_CODEC_SELECT_LOG_LEVEL_INF 1
#define CONFIG_SW_CODEC_SELECT_LOG_LEVEL 3
#define CONFIG_STREAMCTRL_LOG_LEVEL_INF 1
#define CONFIG_STREAMCTRL_LOG_LEVEL 3
#define CONFIG_AUDIO_DATAPATH_LOG_LEVEL_INF 1
#define CONFIG_AUDIO_DATAPATH_LOG_LEVEL 3
#define CONFIG_AUDIO_SYNC_TIMER_LOG_LEVEL_INF 1
#define CONFIG_AUDIO_SYNC_TIMER_LOG_LEVEL 3
#define CONFIG_ENCODER_THREAD_PRIO 3
#define CONFIG_AUDIO_DATAPATH_THREAD_PRIO 4
#define CONFIG_ENCODER_STACK_SIZE 7500
#define CONFIG_AUDIO_DATAPATH_STACK_SIZE 4096
#define CONFIG_BT_AUDIO 1
#define CONFIG_BT_EXT_ADV 1
#define CONFIG_BT_DEVICE_NAME "NRF5340_AUDIO"
#define CONFIG_BT_GATT_CLIENT 1
#define CONFIG_BT_BONDABLE 1
#define CONFIG_BT_PRIVACY 1
#define CONFIG_BT_SCAN_WITH_IDENTITY 1
#define CONFIG_BT_SMP 1
#define CONFIG_BT_L2CAP_TX_BUF_COUNT 12
#define CONFIG_BT_BUF_ACL_RX_SIZE 259
#define CONFIG_SETTINGS 1
#define CONFIG_BT_SETTINGS 1
#define CONFIG_FLASH 1
#define CONFIG_FLASH_MAP 1
#define CONFIG_NVS 1
#define CONFIG_NVS_LOG_LEVEL 2
#define CONFIG_BT_AUDIO_UNICAST_SERVER 1
#define CONFIG_BT_MAX_CONN 1
#define CONFIG_BT_ISO_MAX_CHAN 2
#define CONFIG_BT_ASCS_ASE_SNK_COUNT 1
#define CONFIG_BT_ASCS_ASE_SRC_COUNT 1
#define CONFIG_BT_PERIPHERAL 1
#define CONFIG_BT_PACS_SNK_CONTEXT 0x0007
#define CONFIG_BT_PACS_SRC_CONTEXT 0x0007
#define CONFIG_BT_CSIS 1
#define CONFIG_BT_CAP_ACCEPTOR 1
#define CONFIG_BT_CAP_ACCEPTOR_SET_MEMBER 1
#define CONFIG_BT_ISO_TX_BUF_COUNT 1
#define CONFIG_BT_MAX_PAIRED 1
#define CONFIG_BT_GATT_DYNAMIC_DB 1
#define CONFIG_BT_PAC_SNK 1
#define CONFIG_BLE_ACL_CONN_INTERVAL 80
#define CONFIG_BLE_ACL_SLAVE_LATENCY 0
#define CONFIG_BLE_ACL_SUP_TIMEOUT 400
#define CONFIG_BLE_CONN_TX_POWER_0DBM 1
#define CONFIG_BLE_CONN_TX_POWER_DBM 0
#define CONFIG_BLE_ADV_TX_POWER_0DBM 1
#define CONFIG_BLE_ADV_TX_POWER_DBM 0
#define CONFIG_BLE_ISO_RX_STATS_S 0
#define CONFIG_BT_AUDIO_UNICAST_RECOMMENDED 1
#define CONFIG_BLE_LOG_LEVEL_INF 1
#define CONFIG_BLE_LOG_LEVEL 3
#define CONFIG_AUDIO_SERVICES_LOG_LEVEL_INF 1
#define CONFIG_AUDIO_SERVICES_LOG_LEVEL 3
#define CONFIG_CS47L63_THREAD_PRIO 5
#define CONFIG_CS47L63_STACK_SIZE 700
#define CONFIG_CS47L63_LOG_LEVEL_INF 1
#define CONFIG_CS47L63_LOG_LEVEL 3
#define CONFIG_INA231_LOG_LEVEL_INF 1
#define CONFIG_INA231_LOG_LEVEL 3
#define CONFIG_CTRL_EVENTS_LOG_LEVEL_INF 1
#define CONFIG_CTRL_EVENTS_LOG_LEVEL 3
#define CONFIG_NET_BUF 1
#define CONFIG_NET_BUF_USER_DATA_SIZE 0
#define CONFIG_BUTTON_DEBOUNCE_MS 50
#define CONFIG_POWER_MODULE_MIN_MEAS_TIME_MS 200
#define CONFIG_I2S_LRCK_FREQ_HZ 48000
#define CONFIG_I2S_CH_NUM 2
#define CONFIG_MODULE_AUDIO_USB_LOG_LEVEL_INF 1
#define CONFIG_MODULE_AUDIO_USB_LOG_LEVEL 3
#define CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL_INF 1
#define CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL 3
#define CONFIG_MODULE_DFU_ENTRY_LOG_LEVEL_INF 1
#define CONFIG_MODULE_DFU_ENTRY_LOG_LEVEL 3
#define CONFIG_MODULE_HW_CODEC_LOG_LEVEL_INF 1
#define CONFIG_MODULE_HW_CODEC_LOG_LEVEL 3
#define CONFIG_MODULE_LED_LOG_LEVEL_INF 1
#define CONFIG_MODULE_LED_LOG_LEVEL 3
#define CONFIG_MODULE_POWER_LOG_LEVEL_INF 1
#define CONFIG_MODULE_POWER_LOG_LEVEL 3
#define CONFIG_MODULE_SD_CARD_LOG_LEVEL_INF 1
#define CONFIG_MODULE_SD_CARD_LOG_LEVEL 3
#define CONFIG_POWER_MODULE_THREAD_PRIO 6
#define CONFIG_POWER_MODULE_STACK_SIZE 1152
#define CONFIG_NRFX_NVMC 1
#define CONFIG_FIFO_FRAME_SPLIT_NUM 10
#define CONFIG_FIFO_TX_FRAME_COUNT 3
#define CONFIG_FIFO_RX_FRAME_COUNT 1
#define CONFIG_BOARD_VERSION_LOG_LEVEL_INF 1
#define CONFIG_BOARD_VERSION_LOG_LEVEL 3
#define CONFIG_CHAN_ASSIGNMENT_LOG_LEVEL_INF 1
#define CONFIG_CHAN_ASSIGNMENT_LOG_LEVEL 3
#define CONFIG_CONTIN_ARRAY_LOG_LEVEL_INF 1
#define CONFIG_CONTIN_ARRAY_LOG_LEVEL 3
#define CONFIG_DATA_FIFO_LOG_LEVEL_INF 1
#define CONFIG_DATA_FIFO_LOG_LEVEL 3
#define CONFIG_ERROR_HANDLER_LOG_LEVEL_INF 1
#define CONFIG_ERROR_HANDLER_LOG_LEVEL 3
#define CONFIG_FW_INFO_LOG_LEVEL_INF 1
#define CONFIG_FW_INFO_LOG_LEVEL 3
#define CONFIG_PCM_MIX_LOG_LEVEL_INF 1
#define CONFIG_PCM_MIX_LOG_LEVEL 3
#define CONFIG_PSCM_LOG_LEVEL_INF 1
#define CONFIG_PSCM_LOG_LEVEL 3
#define CONFIG_AUDIO_DFU 0
#define CONFIG_BT_L2CAP_TX_MTU 65
#define CONFIG_BT_BUF_ACL_TX_SIZE 27
#define CONFIG_THREAD_MONITOR 1
#define CONFIG_MAIN_LOG_LEVEL_INF 1
#define CONFIG_MAIN_LOG_LEVEL 3
#define CONFIG_GPIO 1
#define CONFIG_ADC_INIT_PRIORITY 50
#define CONFIG_GPIO_INIT_PRIORITY 40
#define CONFIG_BT_HCI_ACL_FLOW_CONTROL 1
#define CONFIG_BOARD "nrf5340_audio_nrf5340_cpuapp"
#define CONFIG_FLASH_LOAD_SIZE 0x0
#define CONFIG_SRAM_SIZE 448
#define CONFIG_FLASH_LOAD_OFFSET 0x0
#define CONFIG_MBOX_NRFX_IPC 1
#define CONFIG_HEAP_MEM_POOL_SIZE 4096
#define CONFIG_BT_HCI_VS 1
#define CONFIG_BT_ECC 1
#define CONFIG_SOC "nRF5340_CPUAPP_QKAA"
#define CONFIG_SOC_SERIES "nrf53"
#define CONFIG_NUM_IRQS 69
#define CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC 32768
#define CONFIG_CLOCK_CONTROL_INIT_PRIORITY 30
#define CONFIG_FLASH_SIZE 1024
#define CONFIG_FLASH_BASE_ADDRESS 0x0
#define CONFIG_ICACHE_LINE_SIZE 32
#define CONFIG_DCACHE_LINE_SIZE 32
#define CONFIG_ROM_START_OFFSET 0x0
#define CONFIG_PINCTRL 1
#define CONFIG_CLOCK_CONTROL 1
#define CONFIG_SOC_HAS_TIMING_FUNCTIONS 1
#define CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT 1
#define CONFIG_PM 1
#define CONFIG_IPC_SERVICE_BACKEND_RPMSG_SHMEM_RESET 1
#define CONFIG_LOG_DOMAIN_NAME ""
#define CONFIG_NRF_RTC_TIMER 1
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 32768
#define CONFIG_BUILD_OUTPUT_HEX 1
#define CONFIG_SERIAL_INIT_PRIORITY 55
#define CONFIG_TINYCRYPT 1
#define CONFIG_SERIAL 1
#define CONFIG_MP_MAX_NUM_CPUS 1
#define CONFIG_PLATFORM_SPECIFIC_INIT 1
#define CONFIG_HAS_DTS 1
#define CONFIG_DT_HAS_ARM_ARMV8M_ITM_ENABLED 1
#define CONFIG_DT_HAS_ARM_ARMV8M_MPU_ENABLED 1
#define CONFIG_DT_HAS_ARM_CORTEX_M33F_ENABLED 1
#define CONFIG_DT_HAS_ARM_CRYPTOCELL_312_ENABLED 1
#define CONFIG_DT_HAS_ARM_V8M_NVIC_ENABLED 1
#define CONFIG_DT_HAS_FIXED_PARTITIONS_ENABLED 1
#define CONFIG_DT_HAS_GPIO_KEYS_ENABLED 1
#define CONFIG_DT_HAS_GPIO_LEDS_ENABLED 1
#define CONFIG_DT_HAS_MMIO_SRAM_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_MBOX_NRF_IPC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_CC312_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_CLOCK_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_CTRLAPPERI_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_DCNF_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_DPPIC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_EGU_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_FICR_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_GPIO_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_GPIO_FORWARDER_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_GPIOTE_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_I2S_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_IEEE802154_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_IPC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_KMU_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_MUTEX_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_OSCILLATORS_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_PINCTRL_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_POWER_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_PWM_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_REGULATORS_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_RESET_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_RTC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_SAADC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_SPIM_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_SPU_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_TIMER_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_TWIM_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_UARTE_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_UICR_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_USBD_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_USBREG_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_VMC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_WDT_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF53_FLASH_CONTROLLER_ENABLED 1
#define CONFIG_DT_HAS_SOC_NV_FLASH_ENABLED 1
#define CONFIG_DT_HAS_USB_AUDIO_HS_ENABLED 1
#define CONFIG_DT_HAS_ZEPHYR_BT_HCI_ENTROPY_ENABLED 1
#define CONFIG_DT_HAS_ZEPHYR_IPC_OPENAMP_STATIC_VRINGS_ENABLED 1
#define CONFIG_DT_HAS_ZEPHYR_SDHC_SPI_SLOT_ENABLED 1
#define CONFIG_DT_HAS_ZEPHYR_SDMMC_DISK_ENABLED 1
#define CONFIG_NEWLIB_LIBC_NANO 1
#define CONFIG_NUM_METAIRQ_PRIORITIES 0
#define CONFIG_LOG_BUFFER_SIZE 1024
#define CONFIG_INIT_STACKS 1
#define CONFIG_WARN_EXPERIMENTAL 1
#define CONFIG_PRIVILEGED_STACK_SIZE 1024
#define CONFIG_BT_BUF_CMD_TX_COUNT 10
#define CONFIG_INIT_ARCH_HW_AT_BOOT 1
#define CONFIG_LINKER_LAST_SECTION_ID 1
#define CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE 4096
#define CONFIG_LOG_DEFAULT_LEVEL 3
#define CONFIG_PM_PARTITION_SIZE_PROVISION 0x280
#define CONFIG_PM_PARTITION_SIZE_B0_IMAGE 0x8000
#define CONFIG_SB_VALIDATION_INFO_MAGIC 0x86518483
#define CONFIG_SB_VALIDATION_POINTER_MAGIC 0x6919b47e
#define CONFIG_SB_VALIDATION_INFO_CRYPTO_ID 1
#define CONFIG_SB_VALIDATION_INFO_VERSION 2
#define CONFIG_SB_VALIDATION_METADATA_OFFSET 0
#define CONFIG_SB_VALIDATE_FW_SIGNATURE 1
#define CONFIG_BT_HCI_TX_STACK_SIZE 1024
#define CONFIG_BT_RX_STACK_SIZE 2048
#define CONFIG_BT_GATT_AUTO_SEC_REQ 1
#define CONFIG_BT_DRIVERS 1
#define CONFIG_BT_BROADCASTER 1
#define CONFIG_BT_CONN 1
#define CONFIG_BT_PHY_UPDATE 1
#define CONFIG_BT_DATA_LEN_UPDATE 1
#define CONFIG_BT_EXT_ADV_MAX_ADV_SET 1
#define CONFIG_BT_IAS_LOG_LEVEL_INF 1
#define CONFIG_BT_IAS_LOG_LEVEL 3
#define CONFIG_BT_BUF_ACL_TX_COUNT 3
#define CONFIG_BT_BUF_ACL_RX_COUNT 6
#define CONFIG_BT_BUF_EVT_RX_SIZE 68
#define CONFIG_BT_BUF_EVT_RX_COUNT 10
#define CONFIG_BT_BUF_EVT_DISCARDABLE_SIZE 58
#define CONFIG_BT_BUF_EVT_DISCARDABLE_COUNT 3
#define CONFIG_BT_BUF_CMD_TX_SIZE 255
#define CONFIG_BT_HAS_HCI_VS 1
#define CONFIG_BT_RPA 1
#define CONFIG_BT_ASSERT 1
#define CONFIG_BT_ASSERT_VERBOSE 1
#define CONFIG_BT_DEBUG_NONE 1
#define CONFIG_BT_LONG_WQ 1
#define CONFIG_BT_LONG_WQ_STACK_SIZE 1300
#define CONFIG_BT_LONG_WQ_PRIO 10
#define CONFIG_BT_HCI_HOST 1
#define CONFIG_BT_HCI_TX_PRIO 7
#define CONFIG_BT_HCI_RESERVE 1
#define CONFIG_BT_RECV_WORKQ_BT 1
#define CONFIG_BT_RX_PRIO 8
#define CONFIG_BT_DRIVER_RX_HIGH_PRIO 6
#define CONFIG_BT_AUDIO_UNICAST 1
#define CONFIG_BT_CODEC_MAX_DATA_COUNT 5
#define CONFIG_BT_CODEC_MAX_DATA_LEN 4
#define CONFIG_BT_CODEC_MAX_METADATA_COUNT 2
#define CONFIG_BT_CODEC_MAX_METADATA_LEN 4
#define CONFIG_BT_AUDIO_STREAM 1
#define CONFIG_BT_PAC_SNK_LOC 1
#define CONFIG_BT_PAC_SRC 1
#define CONFIG_BT_PAC_SRC_LOC 1
#define CONFIG_BT_PACS 1
#define CONFIG_BT_ASCS 1
#define CONFIG_BT_ASCS_ASE_SNK 1
#define CONFIG_BT_ASCS_ASE_SRC 1
#define CONFIG_BT_VOCS_MAX_INSTANCE_COUNT 0
#define CONFIG_BT_VOCS_CLIENT_MAX_INSTANCE_COUNT 0
#define CONFIG_BT_AICS_MAX_INSTANCE_COUNT 0
#define CONFIG_BT_AICS_CLIENT_MAX_INSTANCE_COUNT 0
#define CONFIG_BT_CSIS_ENC_SIRK_SUPPORT 1
#define CONFIG_BT_CSIS_MAX_INSTANCE_COUNT 1
#define CONFIG_BT_CAP 1
#define CONFIG_BT_HOST_CRYPTO 1
#define CONFIG_BT_HOST_CRYPTO_PRNG 1
#define CONFIG_BT_SETTINGS_CCC_LAZY_LOADING 1
#define CONFIG_BT_SETTINGS_CCC_STORE_ON_WRITE 1
#define CONFIG_BT_SETTINGS_USE_PRINTK 1
#define CONFIG_BT_LIM_ADV_TIMEOUT 30
#define CONFIG_BT_CONN_TX_MAX 12
#define CONFIG_BT_RPA_TIMEOUT 900
#define CONFIG_BT_SMP_ENFORCE_MITM 1
#define CONFIG_BT_SMP_MIN_ENC_KEY_SIZE 7
#define CONFIG_BT_L2CAP_TX_FRAG_COUNT 2
#define CONFIG_BT_ATT_PREPARE_COUNT 0
#define CONFIG_BT_ATT_RETRY_ON_SEC_ERR 1
#define CONFIG_BT_GATT_SERVICE_CHANGED 1
#define CONFIG_BT_GATT_CACHING 1
#define CONFIG_BT_GATT_ENFORCE_SUBSCRIPTION 1
#define CONFIG_BT_GATT_READ_MULTIPLE 1
#define CONFIG_BT_GATT_READ_MULT_VAR_LEN 1
#define CONFIG_BT_CREATE_CONN_TIMEOUT 3
#define CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT 5000
#define CONFIG_BT_DEVICE_APPEARANCE 0
#define CONFIG_BT_ID_MAX 1
#define CONFIG_NRF_CLOUD_CLIENT_ID_SRC_COMPILE_TIME 1
#define CONFIG_NRF_CLOUD_CLIENT_ID "my-client-id"
#define CONFIG_NRF_CLOUD_LOG_LEVEL_INF 1
#define CONFIG_NRF_CLOUD_LOG_LEVEL 3
#define CONFIG_MPSL_FEM_LOG_LEVEL_INF 1
#define CONFIG_MPSL_FEM_LOG_LEVEL 3
#define CONFIG_MPSL_THREAD_COOP_PRIO 8
#define CONFIG_MPSL_WORK_STACK_SIZE 1024
#define CONFIG_MPSL_TIMESLOT_SESSION_COUNT 0
#define CONFIG_MPSL_LOG_LEVEL_INF 1
#define CONFIG_MPSL_LOG_LEVEL 3
#define CONFIG_SRAM_BASE_ADDRESS 0x20000000
#define CONFIG_PM_PARTITION_SIZE_SETTINGS_STORAGE 0x2000
#define CONFIG_PM_PARTITION_ALIGN_SETTINGS_STORAGE 0x4000
#define CONFIG_PM_EXTERNAL_FLASH_BASE 0x0
#define CONFIG_PM_SRAM_BASE 0x20000000
#define CONFIG_PM_SRAM_SIZE 0x80000
#define CONFIG_MGMT_FMFU_LOG_LEVEL_INF 1
#define CONFIG_MGMT_FMFU_LOG_LEVEL 3
#define CONFIG_NRF_SPU_FLASH_REGION_SIZE 0x4000
#define CONFIG_FPROTECT_BLOCK_SIZE 0x4000
#define CONFIG_HW_UNIQUE_KEY_PARTITION_SIZE 0x0
#define CONFIG_HW_CC3XX 1
#define CONFIG_NRFX_GPIOTE_NUM_OF_EVT_HANDLERS 1
#define CONFIG_ZTEST_MULTICORE_DEFAULT_SETTINGS 1
#define CONFIG_ZEPHYR_NRF_MODULE 1
#define CONFIG_ZEPHYR_HOSTAP_MODULE 1
#define CONFIG_BOOT_SIGNATURE_KEY_FILE ""
#define CONFIG_DT_FLASH_WRITE_BLOCK_SIZE 4
#define CONFIG_MCUBOOT_USB_SUPPORT 1
#define CONFIG_ZEPHYR_MCUBOOT_MODULE 1
#define CONFIG_ZEPHYR_MBEDTLS_MODULE 1
#define CONFIG_MBEDTLS_BUILTIN 1
#define CONFIG_ZEPHYR_TRUSTED_FIRMWARE_M_MODULE 1
#define CONFIG_ZEPHYR_CJSON_MODULE 1
#define CONFIG_ZEPHYR_AZURE_SDK_FOR_C_MODULE 1
#define CONFIG_ZEPHYR_MEMFAULT_FIRMWARE_SDK_MODULE 1
#define CONFIG_ZEPHYR_CIRRUS_LOGIC_MODULE 1
#define CONFIG_ZEPHYR_OPENTHREAD_MODULE 1
#define CONFIG_ZEPHYR_CANOPENNODE_MODULE 1
#define CONFIG_ZEPHYR_CHRE_MODULE 1
#define CONFIG_ZEPHYR_HAL_NORDIC_MODULE 1
#define CONFIG_HAS_NORDIC_DRIVERS 1
#define CONFIG_HAS_NRFX 1
#define CONFIG_NRFX_CLOCK_LFXO_TWO_STAGE_ENABLED 1
#define CONFIG_NRFX_GPIOTE 1
#define CONFIG_NRFX_IPC 1
#define CONFIG_NRFX_SPIM 1
#define CONFIG_NRFX_SPIM4 1
#define CONFIG_NRFX_TIMER 1
#define CONFIG_NRFX_TWIM 1
#define CONFIG_NRFX_TWIM1 1
#define CONFIG_ZEPHYR_LIBLC3_MODULE 1
#define CONFIG_ZEPHYR_LITTLEFS_MODULE 1
#define CONFIG_ZEPHYR_LORAMAC_NODE_MODULE 1
#define CONFIG_ZEPHYR_LVGL_MODULE 1
#define CONFIG_ZEPHYR_LZ4_MODULE 1
#define CONFIG_ZEPHYR_NANOPB_MODULE 1
#define CONFIG_ZEPHYR_PICOLIBC_MODULE 1
#define CONFIG_ZEPHYR_TRACERECORDER_MODULE 1
#define CONFIG_ZEPHYR_UOSCORE_UEDHOC_MODULE 1
#define CONFIG_ZEPHYR_ZCBOR_MODULE 1
#define CONFIG_ZEPHYR_ZSCILIB_MODULE 1
#define CONFIG_NRF_MODEM_SHMEM_CTRL_SIZE 0x4e8
#define CONFIG_NRFXLIB_CRYPTO 1
#define CONFIG_CRYPTOCELL_CC312_USABLE 1
#define CONFIG_CRYPTOCELL_USABLE 1
#define CONFIG_NRF_CC3XX_PLATFORM 1
#define CONFIG_CC3XX_MUTEX_LOCK 1
#define CONFIG_NRF_802154_SOURCE_NRFXLIB 1
#define CONFIG_ZEPHYR_NRFXLIB_MODULE 1
#define CONFIG_ZEPHYR_CONNECTEDHOMEIP_MODULE 1
#define CONFIG_HAS_CMSIS_CORE 1
#define CONFIG_HAS_CMSIS_CORE_M 1
#define CONFIG_CMSIS_DSP_BASICMATH 1
#define CONFIG_CMSIS_DSP_FILTERING 1
#define CONFIG_CMSIS_DSP_INTERPOLATION 1
#define CONFIG_CMSIS_DSP_SUPPORT 1
#define CONFIG_CMSIS_DSP_TABLES 1
#define CONFIG_CMSIS_DSP_TABLES_ALL_FAST 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_COS_F32 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_COS_Q31 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_COS_Q15 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_SIN_F32 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_SIN_Q31 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_SIN_Q15 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_SIN_COS_F32 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_SIN_COS_Q31 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_LMS_NORM_Q31 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_LMS_NORM_Q15 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_CMPLX_MAG_Q31 1
#define CONFIG_CMSIS_DSP_TABLES_ARM_CMPLX_MAG_Q15 1
#define CONFIG_CMSIS_DSP_FLOAT16 1
#define CONFIG_LIBMETAL 1
#define CONFIG_LIBMETAL_SRC_PATH "libmetal"
#define CONFIG_OPENAMP 1
#define CONFIG_OPENAMP_SRC_PATH "open-amp"
#define CONFIG_OPENAMP_MASTER 1
#define CONFIG_OPENAMP_SLAVE 1
#define CONFIG_TINYCRYPT_SHA256 1
#define CONFIG_TINYCRYPT_SHA256_HMAC 1
#define CONFIG_TINYCRYPT_SHA256_HMAC_PRNG 1
#define CONFIG_TINYCRYPT_AES 1
#define CONFIG_TINYCRYPT_AES_CMAC 1
#define CONFIG_BOARD_REVISION "$BOARD_REVISION"
#define CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP 1
#define CONFIG_BOARD_ENABLE_DCDC_HV 1
#define CONFIG_DOMAIN_CPUNET_BOARD "nrf5340_audio_dk_nrf5340_cpunet"
#define CONFIG_DOMAIN_CPUAPP_BOARD "nrf5340_audio_dk_nrf5340_cpuapp"
#define CONFIG_SOC_SERIES_NRF53X 1
#define CONFIG_CPU_HAS_ARM_MPU 1
#define CONFIG_CPU_HAS_NRF_IDAU 1
#define CONFIG_NRF_SPU_RAM_REGION_SIZE 0x2000
#define CONFIG_HAS_SWO 1
#define CONFIG_SOC_FAMILY "nordic_nrf"
#define CONFIG_SOC_FAMILY_NRF 1
#define CONFIG_HAS_HW_NRF_CC312 1
#define CONFIG_HAS_HW_NRF_CLOCK 1
#define CONFIG_HAS_HW_NRF_CTRLAP 1
#define CONFIG_HAS_HW_NRF_DCNF 1
#define CONFIG_HAS_HW_NRF_DPPIC 1
#define CONFIG_HAS_HW_NRF_EGU0 1
#define CONFIG_HAS_HW_NRF_EGU1 1
#define CONFIG_HAS_HW_NRF_EGU2 1
#define CONFIG_HAS_HW_NRF_EGU3 1
#define CONFIG_HAS_HW_NRF_EGU4 1
#define CONFIG_HAS_HW_NRF_EGU5 1
#define CONFIG_HAS_HW_NRF_GPIO0 1
#define CONFIG_HAS_HW_NRF_GPIO1 1
#define CONFIG_HAS_HW_NRF_GPIOTE 1
#define CONFIG_HAS_HW_NRF_I2S 1
#define CONFIG_HAS_HW_NRF_IPC 1
#define CONFIG_HAS_HW_NRF_KMU 1
#define CONFIG_HAS_HW_NRF_MUTEX 1
#define CONFIG_HAS_HW_NRF_NVMC_PE 1
#define CONFIG_HAS_HW_NRF_OSCILLATORS 1
#define CONFIG_HAS_HW_NRF_POWER 1
#define CONFIG_HAS_HW_NRF_PWM0 1
#define CONFIG_HAS_HW_NRF_REGULATORS 1
#define CONFIG_HAS_HW_NRF_RESET 1
#define CONFIG_HAS_HW_NRF_RTC0 1
#define CONFIG_HAS_HW_NRF_RTC1 1
#define CONFIG_HAS_HW_NRF_SAADC 1
#define CONFIG_HAS_HW_NRF_SPIM4 1
#define CONFIG_HAS_HW_NRF_SPU 1
#define CONFIG_HAS_HW_NRF_TIMER0 1
#define CONFIG_HAS_HW_NRF_TIMER1 1
#define CONFIG_HAS_HW_NRF_TIMER2 1
#define CONFIG_HAS_HW_NRF_TWIM1 1
#define CONFIG_HAS_HW_NRF_UARTE0 1
#define CONFIG_HAS_HW_NRF_USBD 1
#define CONFIG_HAS_HW_NRF_USBREG 1
#define CONFIG_HAS_HW_NRF_VMC 1
#define CONFIG_HAS_HW_NRF_WDT0 1
#define CONFIG_NRF_HW_RTC1_RESERVED 1
#define CONFIG_SOC_NRF5340_CPUAPP 1
#define CONFIG_SOC_NRF5340_CPUAPP_QKAA 1
#define CONFIG_SOC_DCDC_NRF53X_APP 1
#define CONFIG_SOC_DCDC_NRF53X_NET 1
#define CONFIG_SOC_DCDC_NRF53X_HV 1
#define CONFIG_SOC_NRF_GPIO_FORWARDER_FOR_NRF5340 1
#define CONFIG_SOC_ENABLE_LFXO 1
#define CONFIG_SOC_LFXO_CAP_INT_7PF 1
#define CONFIG_SOC_HFXO_CAP_DEFAULT 1
#define CONFIG_NRF_ENABLE_CACHE 1
#define CONFIG_NRF53_SYNC_RTC 1
#define CONFIG_SYNC_RTC_LOG_LEVEL_INF 1
#define CONFIG_SYNC_RTC_LOG_LEVEL 3
#define CONFIG_NRF53_SYNC_RTC_INIT_PRIORITY 90
#define CONFIG_NRF_RTC_TIMER_USER_CHAN_COUNT 1
#define CONFIG_NRF53_SYNC_RTC_LOG_TIMESTAMP 1
#define CONFIG_NRF53_SYNC_RTC_IPM_OUT 7
#define CONFIG_NRF53_SYNC_RTC_IPM_IN 8
#define CONFIG_IPM_MSG_CH_8_ENABLE 1
#define CONFIG_IPM_MSG_CH_8_RX 1
#define CONFIG_NRF_SOC_SECURE_SUPPORTED 1
#define CONFIG_NRF_APPROTECT_USE_UICR 1
#define CONFIG_NRF_SECURE_APPROTECT_USE_UICR 1
#define CONFIG_SOC_LOG_LEVEL_INF 1
#define CONFIG_SOC_LOG_LEVEL 3
#define CONFIG_SOC_COMPATIBLE_NRF 1
#define CONFIG_ARCH "arm"
#define CONFIG_CPU_CORTEX 1
#define CONFIG_CPU_CORTEX_M 1
#define CONFIG_ISA_THUMB2 1
#define CONFIG_ASSEMBLER_ISA_THUMB2 1
#define CONFIG_COMPILER_ISA_THUMB2 1
#define CONFIG_STACK_ALIGN_DOUBLE_WORD 1
#define CONFIG_FAULT_DUMP 2
#define CONFIG_BUILTIN_STACK_GUARD 1
#define CONFIG_ARM_STACK_PROTECTION 1
#define CONFIG_ARM_STORE_EXC_RETURN 1
#define CONFIG_FP_HARDABI 1
#define CONFIG_FP16 1
#define CONFIG_FP16_IEEE 1
#define CONFIG_CPU_CORTEX_M33 1
#define CONFIG_CPU_CORTEX_M_HAS_SYSTICK 1
#define CONFIG_CPU_CORTEX_M_HAS_DWT 1
#define CONFIG_CPU_CORTEX_M_HAS_BASEPRI 1
#define CONFIG_CPU_CORTEX_M_HAS_VTOR 1
#define CONFIG_CPU_CORTEX_M_HAS_SPLIM 1
#define CONFIG_CPU_CORTEX_M_HAS_PROGRAMMABLE_FAULT_PRIOS 1
#define CONFIG_CPU_CORTEX_M_HAS_CMSE 1
#define CONFIG_ARMV7_M_ARMV8_M_MAINLINE 1
#define CONFIG_ARMV8_M_MAINLINE 1
#define CONFIG_ARMV8_M_SE 1
#define CONFIG_ARMV7_M_ARMV8_M_FP 1
#define CONFIG_ARMV8_M_DSP 1
#define CONFIG_GEN_ISR_TABLES 1
#define CONFIG_NULL_POINTER_EXCEPTION_DETECTION_NONE 1
#define CONFIG_ARM_TRUSTZONE_M 1
#define CONFIG_GEN_IRQ_VECTOR_TABLE 1
#define CONFIG_ARM_MPU 1
#define CONFIG_ARM_MPU_REGION_MIN_ALIGN_AND_SIZE 32
#define CONFIG_MPU_ALLOW_FLASH_WRITE 1
#define CONFIG_CUSTOM_SECTION_MIN_ALIGN_SIZE 32
#define CONFIG_ARM 1
#define CONFIG_ARCH_IS_SET 1
#define CONFIG_ARCH_LOG_LEVEL_INF 1
#define CONFIG_ARCH_LOG_LEVEL 3
#define CONFIG_LITTLE_ENDIAN 1
#define CONFIG_HW_STACK_PROTECTION 1
#define CONFIG_KOBJECT_TEXT_AREA 256
#define CONFIG_KOBJECT_DATA_AREA_RESERVE_EXTRA_PERCENT 100
#define CONFIG_KOBJECT_RODATA_AREA_EXTRA_BYTES 16
#define CONFIG_GEN_PRIV_STACKS 1
#define CONFIG_ARCH_IRQ_VECTOR_TABLE_ALIGN 4
#define CONFIG_IRQ_VECTOR_TABLE_JUMP_BY_ADDRESS 1
#define CONFIG_GEN_SW_ISR_TABLE 1
#define CONFIG_ARCH_SW_ISR_TABLE_ALIGN 4
#define CONFIG_GEN_IRQ_START_VECTOR 0
#define CONFIG_ARCH_HAS_SINGLE_THREAD_SUPPORT 1
#define CONFIG_ARCH_HAS_TIMING_FUNCTIONS 1
#define CONFIG_ARCH_HAS_TRUSTED_EXECUTION 1
#define CONFIG_ARCH_HAS_STACK_PROTECTION 1
#define CONFIG_ARCH_HAS_USERSPACE 1
#define CONFIG_ARCH_HAS_EXECUTABLE_PAGE_BIT 1
#define CONFIG_ARCH_HAS_RAMFUNC_SUPPORT 1
#define CONFIG_ARCH_HAS_NESTED_EXCEPTION_DETECTION 1
#define CONFIG_ARCH_SUPPORTS_COREDUMP 1
#define CONFIG_ARCH_SUPPORTS_ARCH_HW_INIT 1
#define CONFIG_ARCH_HAS_EXTRA_EXCEPTION_INFO 1
#define CONFIG_ARCH_HAS_THREAD_LOCAL_STORAGE 1
#define CONFIG_ARCH_HAS_SUSPEND_TO_RAM 1
#define CONFIG_ARCH_HAS_THREAD_ABORT 1
#define CONFIG_ARCH_HAS_CODE_DATA_RELOCATION 1
#define CONFIG_CPU_HAS_TEE 1
#define CONFIG_CPU_HAS_FPU 1
#define CONFIG_CPU_HAS_MPU 1
#define CONFIG_MPU 1
#define CONFIG_MPU_LOG_LEVEL_INF 1
#define CONFIG_MPU_LOG_LEVEL 3
#define CONFIG_MPU_REQUIRES_NON_OVERLAPPING_REGIONS 1
#define CONFIG_MPU_GAP_FILLING 1
#define CONFIG_SRAM_REGION_PERMISSIONS 1
#define CONFIG_TOOLCHAIN_HAS_BUILTIN_FFS 1
#define CONFIG_KERNEL_LOG_LEVEL_INF 1
#define CONFIG_KERNEL_LOG_LEVEL 3
#define CONFIG_MULTITHREADING 1
#define CONFIG_NUM_COOP_PRIORITIES 16
#define CONFIG_NUM_PREEMPT_PRIORITIES 15
#define CONFIG_COOP_ENABLED 1
#define CONFIG_PREEMPT_ENABLED 1
#define CONFIG_PRIORITY_CEILING -127
#define CONFIG_IDLE_STACK_SIZE 320
#define CONFIG_ISR_STACK_SIZE 2048
#define CONFIG_THREAD_STACK_INFO 1
#define CONFIG_ERRNO 1
#define CONFIG_SCHED_DUMB 1
#define CONFIG_WAITQ_DUMB 1
#define CONFIG_BOOT_BANNER 1
#define CONFIG_BOOT_DELAY 0
#define CONFIG_THREAD_MAX_NAME_LEN 32
#define CONFIG_INSTRUMENT_THREAD_SWITCHING 1
#define CONFIG_THREAD_RUNTIME_STATS 1
#define CONFIG_SCHED_THREAD_USAGE 1
#define CONFIG_SCHED_THREAD_USAGE_ALL 1
#define CONFIG_SCHED_THREAD_USAGE_AUTO_ENABLE 1
#define CONFIG_SYSTEM_WORKQUEUE_PRIORITY -1
#define CONFIG_ATOMIC_OPERATIONS_BUILTIN 1
#define CONFIG_TIMESLICING 1
#define CONFIG_TIMESLICE_SIZE 0
#define CONFIG_TIMESLICE_PRIORITY 0
#define CONFIG_POLL 1
#define CONFIG_NUM_MBOX_ASYNC_MSGS 10
#define CONFIG_KERNEL_MEM_POOL 1
#define CONFIG_ARCH_HAS_CUSTOM_SWAP_TO_MAIN 1
#define CONFIG_SWAP_NONATOMIC 1
#define CONFIG_SYS_CLOCK_EXISTS 1
#define CONFIG_TIMEOUT_64BIT 1
#define CONFIG_SYS_CLOCK_MAX_TIMEOUT_DAYS 365
#define CONFIG_XIP 1
#define CONFIG_KERNEL_INIT_PRIORITY_OBJECTS 30
#define CONFIG_KERNEL_INIT_PRIORITY_DEFAULT 40
#define CONFIG_KERNEL_INIT_PRIORITY_DEVICE 50
#define CONFIG_APPLICATION_INIT_PRIORITY 90
#define CONFIG_MP_NUM_CPUS 1
#define CONFIG_TICKLESS_KERNEL 1
#define CONFIG_TOOLCHAIN_SUPPORTS_THREAD_LOCAL_STORAGE 1
#define CONFIG_BT_RPMSG 1
#define CONFIG_CONSOLE 1
#define CONFIG_CONSOLE_INPUT_MAX_LINE_LEN 128
#define CONFIG_CONSOLE_HAS_DRIVER 1
#define CONFIG_CONSOLE_INIT_PRIORITY 60
#define CONFIG_UART_CONSOLE 1
#define CONFIG_UART_CONSOLE_INPUT_EXPIRED 1
#define CONFIG_UART_CONSOLE_INPUT_EXPIRED_TIMEOUT 15000
#define CONFIG_RTT_CONSOLE 1
#define CONFIG_RTT_TX_RETRY_CNT 2
#define CONFIG_RTT_TX_RETRY_DELAY_MS 2
#define CONFIG_UART_CONSOLE_LOG_LEVEL_INF 1
#define CONFIG_UART_CONSOLE_LOG_LEVEL 3
#define CONFIG_HAS_SEGGER_RTT 1
#define CONFIG_USE_SEGGER_RTT 1
#define CONFIG_SEGGER_RTT_MAX_NUM_UP_BUFFERS 3
#define CONFIG_SEGGER_RTT_MAX_NUM_DOWN_BUFFERS 3
#define CONFIG_SEGGER_RTT_BUFFER_SIZE_UP 1024
#define CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN 16
#define CONFIG_SEGGER_RTT_PRINTF_BUFFER_SIZE 64
#define CONFIG_SEGGER_RTT_MODE_NO_BLOCK_SKIP 1
#define CONFIG_SEGGER_RTT_MODE 0
#define CONFIG_SEGGER_RTT_SECTION_NONE 1
#define CONFIG_ETH_INIT_PRIORITY 80
#define CONFIG_SERIAL_HAS_DRIVER 1
#define CONFIG_SERIAL_SUPPORT_ASYNC 1
#define CONFIG_SERIAL_SUPPORT_INTERRUPT 1
#define CONFIG_UART_LOG_LEVEL_INF 1
#define CONFIG_UART_LOG_LEVEL 3
#define CONFIG_UART_USE_RUNTIME_CONFIGURE 1
#define CONFIG_UART_NRFX 1
#define CONFIG_UART_0_NRF_UARTE 1
#define CONFIG_UART_0_ENHANCED_POLL_OUT 1
#define CONFIG_UART_0_NRF_TX_BUFFER_SIZE 32
#define CONFIG_UART_ENHANCED_POLL_OUT 1
#define CONFIG_NRF_UARTE_PERIPHERAL 1
#define CONFIG_INTC_INIT_PRIORITY 40
#define CONFIG_INTC_LOG_LEVEL_INF 1
#define CONFIG_INTC_LOG_LEVEL 3
#define CONFIG_SYSTEM_CLOCK_INIT_PRIORITY 0
#define CONFIG_TICKLESS_CAPABLE 1
#define CONFIG_SYSTEM_CLOCK_WAIT_FOR_STABILITY 1
#define CONFIG_GPIO_LOG_LEVEL_INF 1
#define CONFIG_GPIO_LOG_LEVEL 3
#define CONFIG_GPIO_NRFX 1
#define CONFIG_FXL6408_LOG_LEVEL_INF 1
#define CONFIG_FXL6408_LOG_LEVEL 3
#define CONFIG_SPI_INIT_PRIORITY 70
#define CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE 200
#define CONFIG_SPI_LOG_LEVEL_INF 1
#define CONFIG_SPI_LOG_LEVEL 3
#define CONFIG_SPI_NRFX 1
#define CONFIG_SPI_4_NRF_SPIM 1
#define CONFIG_SDHC 1
#define CONFIG_SDHC_BUFFER_ALIGNMENT 1
#define CONFIG_SPI_SDHC 1
#define CONFIG_SDHC_INIT_PRIORITY 85
#define CONFIG_SDHC_SUPPORTS_SPI_MODE 1
#define CONFIG_SDHC_LOG_LEVEL_INF 1
#define CONFIG_SDHC_LOG_LEVEL 3
#define CONFIG_I2C_NRFX 1
#define CONFIG_I2C_1_NRF_TWIM 1
#define CONFIG_I2C_INIT_PRIORITY 50
#define CONFIG_I2C_LOG_LEVEL_INF 1
#define CONFIG_I2C_LOG_LEVEL 3
#define CONFIG_ADC_SHELL 1
#define CONFIG_ADC_CONFIGURABLE_INPUTS 1
#define CONFIG_ADC_LOG_LEVEL_INF 1
#define CONFIG_ADC_LOG_LEVEL 3
#define CONFIG_ADC_NRFX_SAADC 1
#define CONFIG_CLOCK_CONTROL_LOG_LEVEL_INF 1
#define CONFIG_CLOCK_CONTROL_LOG_LEVEL 3
#define CONFIG_CLOCK_CONTROL_NRF 1
#define CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL 1
#define CONFIG_CLOCK_CONTROL_NRF_K32SRC_50PPM 1
#define CONFIG_CLOCK_CONTROL_NRF_ACCURACY 50
#define CONFIG_FLASH_HAS_DRIVER_ENABLED 1
#define CONFIG_FLASH_HAS_PAGE_LAYOUT 1
#define CONFIG_FLASH_LOG_LEVEL_INF 1
#define CONFIG_FLASH_LOG_LEVEL 3
#define CONFIG_FLASH_PAGE_LAYOUT 1
#define CONFIG_FLASH_INIT_PRIORITY 50
#define CONFIG_SOC_FLASH_NRF 1
#define CONFIG_SOC_FLASH_NRF_RADIO_SYNC_NONE 1
#define CONFIG_SDMMC_INIT_PRIORITY 90
#define CONFIG_SDMMC_VOLUME_NAME "SD"
#define CONFIG_SDMMC_SUBSYS 1
#define CONFIG_SDMMC_LOG_LEVEL_INF 1
#define CONFIG_SDMMC_LOG_LEVEL 3
#define CONFIG_PINCTRL_LOG_LEVEL_INF 1
#define CONFIG_PINCTRL_LOG_LEVEL 3
#define CONFIG_PINCTRL_STORE_REG 1
#define CONFIG_PINCTRL_NRF 1
#define CONFIG_MBOX 1
#define CONFIG_MBOX_INIT_PRIORITY 50
#define CONFIG_MBOX_LOG_LEVEL_INF 1
#define CONFIG_MBOX_LOG_LEVEL 3
#define CONFIG_USBC_LOG_LEVEL_INF 1
#define CONFIG_USBC_LOG_LEVEL 3
#define CONFIG_SUPPORT_MINIMAL_LIBC 1
#define CONFIG_PICOLIBC_SUPPORTED 1
#define CONFIG_NEWLIB_LIBC 1
#define CONFIG_HAS_NEWLIB_LIBC_NANO 1
#define CONFIG_NEWLIB_LIBC_MIN_REQUIRED_HEAP_SIZE 2048
#define CONFIG_NEWLIB_LIBC_FLOAT_PRINTF 1
#define CONFIG_STDOUT_CONSOLE 1
#define CONFIG_RING_BUFFER 1
#define CONFIG_NOTIFY 1
#define CONFIG_MPSC_PBUF 1
#define CONFIG_ONOFF 1
#define CONFIG_CBPRINTF_COMPLETE 1
#define CONFIG_CBPRINTF_FULL_INTEGRAL 1
#define CONFIG_CBPRINTF_FP_SUPPORT 1
#define CONFIG_CBPRINTF_N_SPECIFIER 1
#define CONFIG_CBPRINTF_PACKAGE_LOG_LEVEL_INF 1
#define CONFIG_CBPRINTF_PACKAGE_LOG_LEVEL 3
#define CONFIG_SYS_HEAP_ALLOC_LOOPS 3
#define CONFIG_SYS_HEAP_AUTO 1
#define CONFIG_POSIX_MAX_FDS 4
#define CONFIG_MAX_TIMER_COUNT 5
#define CONFIG_BT_LOG_LEVEL_INF 1
#define CONFIG_BT_LOG_LEVEL 3
#define CONFIG_BT_HCI 1
#define CONFIG_BT_CONN_TX 1
#define CONFIG_BT_ISO 1
#define CONFIG_BT_ISO_UNICAST 1
#define CONFIG_BT_ISO_PERIPHERAL 1
#define CONFIG_BT_ISO_TX_FRAG_COUNT 2
#define CONFIG_BT_ISO_TX_MTU 251
#define CONFIG_BT_ISO_RX_BUF_COUNT 1
#define CONFIG_BT_ISO_RX_MTU 251
#define CONFIG_BT_ISO_MAX_CIG 1
#define CONFIG_BT_COMPANY_ID 0x05F1
#define CONFIG_DEBUG 1
#define CONFIG_STACK_USAGE 1
#define CONFIG_STACK_SENTINEL 1
#define CONFIG_PRINTK 1
#define CONFIG_EARLY_CONSOLE 1
#define CONFIG_ASSERT 1
#define CONFIG_ASSERT_LEVEL 2
#define CONFIG_SPIN_VALIDATE 1
#define CONFIG_ASSERT_VERBOSE 1
#define CONFIG_DEBUG_INFO 1
#define CONFIG_EXCEPTION_STACK_TRACE 1
#define CONFIG_DISK_ACCESS 1
#define CONFIG_DISK_LOG_LEVEL_INF 1
#define CONFIG_DISK_LOG_LEVEL 3
#define CONFIG_FS_LOG_LEVEL_INF 1
#define CONFIG_FS_LOG_LEVEL 3
#define CONFIG_APP_LINK_WITH_FS 1
#define CONFIG_FILE_SYSTEM_MAX_TYPES 2
#define CONFIG_FILE_SYSTEM_MAX_FILE_NAME -1
#define CONFIG_FS_FATFS_MKFS 1
#define CONFIG_FS_FATFS_MOUNT_MKFS 1
#define CONFIG_FS_FATFS_MAX_ROOT_ENTRIES 512
#define CONFIG_FS_FATFS_NUM_FILES 4
#define CONFIG_FS_FATFS_NUM_DIRS 4
#define CONFIG_FS_FATFS_LFN_MODE_STACK 1
#define CONFIG_FS_FATFS_CODEPAGE 437
#define CONFIG_FS_FATFS_MAX_SS 512
#define CONFIG_FS_FATFS_WINDOW_ALIGNMENT 1
#define CONFIG_NVS_LOG_LEVEL_INF 1
#define CONFIG_IPC_SERVICE 1
#define CONFIG_IPC_SERVICE_REG_BACKEND_PRIORITY 46
#define CONFIG_IPC_SERVICE_BACKEND_RPMSG 1
#define CONFIG_IPC_SERVICE_RPMSG 1
#define CONFIG_IPC_SERVICE_STATIC_VRINGS 1
#define CONFIG_IPC_SERVICE_STATIC_VRINGS_ALIGNMENT 4
#define CONFIG_IPC_SERVICE_BACKEND_RPMSG_WQ_STACK_SIZE 1024
#define CONFIG_IPC_SERVICE_BACKEND_RPMSG_NUM_ENDPOINTS_PER_INSTANCE 2
#define CONFIG_IPC_SERVICE_LOG_LEVEL_INF 1
#define CONFIG_IPC_SERVICE_LOG_LEVEL 3
#define CONFIG_LOG 1
#define CONFIG_LOG_MODE_DEFERRED 1
#define CONFIG_LOG_RUNTIME_FILTERING 1
#define CONFIG_LOG_OVERRIDE_LEVEL 0
#define CONFIG_LOG_MAX_LEVEL 4
#define CONFIG_LOG_MODE_OVERFLOW 1
#define CONFIG_LOG_PROCESS_TRIGGER_THRESHOLD 10
#define CONFIG_LOG_PROCESS_THREAD 1
#define CONFIG_LOG_PROCESS_THREAD_STARTUP_DELAY_MS 0
#define CONFIG_LOG_PROCESS_THREAD_SLEEP_MS 1000
#define CONFIG_LOG_PROCESS_THREAD_STACK_SIZE 2048
#define CONFIG_LOG_TRACE_SHORT_TIMESTAMP 1
#define CONFIG_LOG_FUNC_NAME_PREFIX_DBG 1
#define CONFIG_LOG_BACKEND_SHOW_COLOR 1
#define CONFIG_LOG_TAG_MAX_LEN 2
#define CONFIG_LOG_TAG_DEFAULT "--"
#define CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP 1
#define CONFIG_LOG_BACKEND_UART 1
#define CONFIG_LOG_BACKEND_UART_BUFFER_SIZE 1
#define CONFIG_LOG_BACKEND_UART_OUTPUT_TEXT 1
#define CONFIG_LOG_BACKEND_UART_OUTPUT_DEFAULT 0
#define CONFIG_LOG_DOMAIN_ID 0
#define CONFIG_LOG_CMDS 1
#define CONFIG_LOG_USE_VLA 1
#define CONFIG_LOG_MEM_UTILIZATION 1
#define CONFIG_LOG_FAILURE_REPORT_PERIOD 1000
#define CONFIG_LOG_OUTPUT 1
#define CONFIG_NET_BUF_LOG_LEVEL_INF 1
#define CONFIG_NET_BUF_LOG_LEVEL 3
#define CONFIG_PM_LOG_LEVEL_INF 1
#define CONFIG_PM_LOG_LEVEL 3
#define CONFIG_PM_POLICY_DEFAULT 1
#define CONFIG_SHELL 1
#define CONFIG_SHELL_LOG_LEVEL_INF 1
#define CONFIG_SHELL_LOG_LEVEL 3
#define CONFIG_SHELL_BACKENDS 1
#define CONFIG_SHELL_BACKEND_RTT 1
#define CONFIG_SHELL_PROMPT_RTT "rtt:~$ "
#define CONFIG_SHELL_RTT_RX_POLL_PERIOD 10
#define CONFIG_SHELL_BACKEND_RTT_LOG_MESSAGE_QUEUE_TIMEOUT 100
#define CONFIG_SHELL_BACKEND_RTT_LOG_MESSAGE_QUEUE_SIZE 10
#define CONFIG_SHELL_RTT_INIT_LOG_LEVEL_NONE 1
#define CONFIG_SHELL_RTT_INIT_LOG_LEVEL 0
#define CONFIG_SHELL_RTT_LOG_LEVEL_INF 1
#define CONFIG_SHELL_RTT_LOG_LEVEL 3
#define CONFIG_SHELL_STACK_SIZE 1024
#define CONFIG_SHELL_BACKSPACE_MODE_DELETE 1
#define CONFIG_SHELL_CMD_BUFF_SIZE 128
#define CONFIG_SHELL_PRINTF_BUFF_SIZE 30
#define CONFIG_SHELL_DEFAULT_TERMINAL_WIDTH 80
#define CONFIG_SHELL_DEFAULT_TERMINAL_HEIGHT 24
#define CONFIG_SHELL_ARGC_MAX 20
#define CONFIG_SHELL_TAB 1
#define CONFIG_SHELL_TAB_AUTOCOMPLETION 1
#define CONFIG_SHELL_ECHO_STATUS 1
#define CONFIG_SHELL_VT100_COMMANDS 1
#define CONFIG_SHELL_VT100_COLORS 1
#define CONFIG_SHELL_METAKEYS 1
#define CONFIG_SHELL_HELP 1
#define CONFIG_SHELL_HELP_OPT_PARSE 1
#define CONFIG_SHELL_HISTORY 1
#define CONFIG_SHELL_HISTORY_BUFFER 512
#define CONFIG_SHELL_CMD_ROOT ""
#define CONFIG_SHELL_LOG_BACKEND 1
#define CONFIG_SHELL_LOG_FORMAT_TIMESTAMP 1
#define CONFIG_KERNEL_SHELL 1
#define CONFIG_KERNEL_SHELL_REBOOT_DELAY 0
#define CONFIG_DEVMEM_SHELL 1
#define CONFIG_SD_STACK 1
#define CONFIG_SD_LOG_LEVEL_OFF 1
#define CONFIG_SD_LOG_LEVEL 0
#define CONFIG_SD_INIT_TIMEOUT 1500
#define CONFIG_SD_RETRY_COUNT 10
#define CONFIG_SD_OCR_RETRY_COUNT 1000
#define CONFIG_SD_CMD_TIMEOUT 200
#define CONFIG_SD_DATA_TIMEOUT 10000
#define CONFIG_SD_BUFFER_SIZE 64
#define CONFIG_SD_DATA_RETRIES 3
#define CONFIG_SETTINGS_LOG_LEVEL_INF 1
#define CONFIG_SETTINGS_LOG_LEVEL 3
#define CONFIG_SETTINGS_DYNAMIC_HANDLERS 1
#define CONFIG_SETTINGS_NVS 1
#define CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT 1
#define CONFIG_SETTINGS_NVS_SECTOR_COUNT 8
#define CONFIG_TOOLCHAIN_ZEPHYR_0_15 1
#define CONFIG_TOOLCHAIN_ZEPHYR_SUPPORTS_THREAD_LOCAL_STORAGE 1
#define CONFIG_LINKER_ORPHAN_SECTION_WARN 1
#define CONFIG_HAS_FLASH_LOAD_OFFSET 1
#define CONFIG_LD_LINKER_SCRIPT_SUPPORTED 1
#define CONFIG_LD_LINKER_TEMPLATE 1
#define CONFIG_KERNEL_ENTRY "__start"
#define CONFIG_LINKER_SORT_BY_ALIGNMENT 1
#define CONFIG_SRAM_OFFSET 0x0
#define CONFIG_LINKER_GENERIC_SECTIONS_PRESENT_AT_BOOT 1
#define CONFIG_LINKER_LAST_SECTION_ID_PATTERN 0xE015E015
#define CONFIG_DEBUG_OPTIMIZATIONS 1
#define CONFIG_COMPILER_COLOR_DIAGNOSTICS 1
#define CONFIG_FORTIFY_SOURCE_COMPILE_TIME 1
#define CONFIG_COMPILER_OPT ""
#define CONFIG_RUNTIME_ERROR_CHECKS 1
#define CONFIG_KERNEL_BIN_NAME "zephyr"
#define CONFIG_OUTPUT_STAT 1
#define CONFIG_OUTPUT_DISASSEMBLY 1
#define CONFIG_OUTPUT_PRINT_MEMORY_USAGE 1
#define CONFIG_BUILD_OUTPUT_BIN 1
#define CONFIG_WARN_DEPRECATED 1
#define CONFIG_EXPERIMENTAL 1
#define CONFIG_ENFORCE_ZEPHYR_STDINT 1
#define CONFIG_COMPAT_INCLUDES 1
