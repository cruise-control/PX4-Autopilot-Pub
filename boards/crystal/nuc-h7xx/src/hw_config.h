/*
 * hw_config.h — Crystal NUC-H7xx (Nucleo-H753ZI) bootloader configuration
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

/* Boot device selection flags */
#define USB0_DEV       0x01
#define SERIAL0_DEV    0x02
#define SERIAL1_DEV    0x04

/* Firmware starts immediately after the 128 KB bootloader sector */
#define APP_LOAD_ADDRESS               0x08020000

/* Stay in bootloader for 5 s when entered by request or USB power detected */
#define BOOTLOADER_DELAY               5000

/* USB flashing via CDC-ACM (/dev/ttyACM0) */
#define INTERFACE_USB                  1
#define INTERFACE_USB_CONFIG           "/dev/ttyACM0"
#define BOARD_VBUS                     MK_GPIO_INPUT(GPIO_OTGFS_VBUS)

/* UART flashing via USART3 = ST-LINK VCP.
 * With CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y, USART3 is always /dev/ttyS2.
 */
#define INTERFACE_USART                1
#define INTERFACE_USART_CONFIG         "/dev/ttyS2,115200"

/* Bootloader delay signature address (from linker script) */
#define BOOT_DELAY_ADDRESS             0x000001a0

/* Must match board_id in firmware.prototype */
#define BOARD_TYPE                     210

/* STM32H753ZI: 2 MB flash, 16 sectors (0–15), 128 KB each.
 * Flash size read from the device's flash-size register. */
#define _FLASH_KBYTES                  (*(uint32_t *)0x1FF1E880)
#define BOARD_FLASH_SECTORS            (15)
#define BOARD_FLASH_SIZE               (_FLASH_KBYTES * 1024)

/* No sectors reserved for application use beyond the bootloader sector */
#define APP_RESERVATION_SIZE           0

/* HSE oscillator frequency in MHz */
#define OSC_FREQ                       8

/* Bootloader activity LEDs — Nucleo-H753ZI LEDs are ACTIVE HIGH */
#define BOARD_PIN_LED_ACTIVITY         GPIO_LED_YELLOW   /* Yellow LD2 during flash */
#define BOARD_PIN_LED_BOOTLOADER       GPIO_LED_GREEN    /* Green  LD1 while in BL  */
#define BOARD_LED_ON                   1                 /* active HIGH             */
#define BOARD_LED_OFF                  0

/* Disable break-detect to prevent accidental BL entry from idle serial line */
#define SERIAL_BREAK_DETECT_DISABLED   1

/* ---- defaults filled in by the common bootloader header ---- */

#if !defined(ARCH_SN_MAX_LENGTH)
#  define ARCH_SN_MAX_LENGTH 12
#endif

#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 1
#endif

#if !defined(USB_DATA_ALIGN)
#  define USB_DATA_ALIGN
#endif

#ifndef BOOT_DEVICES_SELECTION
#  define BOOT_DEVICES_SELECTION (USB0_DEV | SERIAL0_DEV)
#endif

#ifndef BOOT_DEVICES_FILTER_ONUSB
#  define BOOT_DEVICES_FILTER_ONUSB (USB0_DEV | SERIAL0_DEV)
#endif

#endif /* HW_CONFIG_H_ */
