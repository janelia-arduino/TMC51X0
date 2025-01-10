/* Private defines -----------------------------------------------------------*/
#if defined(ARDUINO_ARCH_STM32)
#define RADEC_CLK_Pin       GPIO_PIN_8
#define RADEC_CLK_GPIO_Port GPIOA
#define RADEC_EN_Pin        GPIO_PIN_4
#define RADEC_EN_GPIO_Port  GPIOA
#define RA_CSN_Pin          GPIO_PIN_0
#define RA_CSN_GPIO_Port    GPIOB
#define RA_MODE_Pin         GPIO_PIN_5
#define RA_MODE_GPIO_Port   GPIOC
#define RA_DIR_Pin          GPIO_PIN_4
#define RA_DIR_GPIO_Port    GPIOC
#define RA_STEP_Pin         GPIO_PIN_10
#define RA_STEP_GPIO_Port   GPIOB
#define RA_DIAG0_Pin        GPIO_PIN_5
#define RA_DIAG0_GPIO_Port  GPIOF
#define RA_DIAG1_Pin        GPIO_PIN_4
#define RA_DIAG1_GPIO_Port  GPIOF
#define RA_HOME_Pin         GPIO_PIN_10
#define RA_HOME_GPIO_Port   GPIOC
#define RA_WORM_Pin         GPIO_PIN_11
#define RA_WORM_GPIO_Port   GPIOC
#define RA_GEAR_Pin         PIO_PIN_12
#define RA_GEAR_GPIO_Port   GPIOC
#define RA_ENC1_Pin         GPIO_PIN_3
#define RA_ENC1_GPIO_Port   GPIOC
#define RA_ENC4_Pin         GPIO_PIN_2
#define RA_ENC4_GPIO_Port   GPIOC
#define HiResEnc2Y_Pin      GPIO_PIN_5
#define HiResEnc2Y_GPIO_Port GPIOB
#define HiResEnc3Y_Pin      GPIO_PIN_1
#define HiResEnc3Y_GPIO_Port GPIOA
#define HiResEnc4Y_Pin      GPIO_PIN_0
#define HiResEnc4Y_GPIO_Port GPIOA
#define DEC_CSN_Pin         GPIO_PIN_12
#define DEC_CSN_GPIO_Port   GPIOB
#define DEC_MODE_Pin        GPIO_PIN_2
#define DEC_MODE_GPIO_Port  GPIOB
#define DEC_DIR_Pin         GPIO_PIN_7
#define DEC_DIR_GPIO_Port   GPIOC
#define DEC_STEP_Pin        GPIO_PIN_11
#define DEC_STEP_GPIO_Port  GPIOB
#define DEC_DIAG0_Pin       GPIO_PIN_8
#define DEC_DIAG0_GPIO_Port GPIOC
#define DEC_DIAG1_Pin       GPIO_PIN_9
#define DEC_DIAG1_GPIO_Port GPIOC
#define DEC_HOME_Pin        GPIO_PIN_15
#define DEC_HOME_GPIO_Port  GPIOC
#define DEC_WORM_Pin        GPIO_PIN_13
#define DEC_WORM_GPIO_Port  GPIOC
#define DEC_GEAR_Pin        GPIO_PIN_14
#define DEC_GEAR_GPIO_Port  GPIOC
#define DEC_ENC1_Pin        GPIO_PIN_1
#define DEC_ENC1_GPIO_Port  GPIOC
#define DEC_ENC4_Pin        GPIO_PIN_0
#define DEC_ENC4_GPIO_Port  GPIOC
#define SNAP_Pin            GPIO_PIN_2
#define SNAP_GPIO_Port      GPIOD

#define LED_POLAR_Pin       GPIO_PIN_9
#define LED_POLAR_GPIO_Port GPIOA
#define V12SENSE_Pin        GPIO_PIN_1
#define V12SENSE_GPIO_Port  GPIOB
#define LED_POWER_Pin       GPIO_PIN_15
#define LED_POWER_GPIO_Port GPIOA
#define VMBOOST_Pin         GPIO_PIN_10
#define VMBOOST_GPIO_Port   GPIOA
#define SW1_Pin             GPIO_PIN_11
#define SW1_GPIO_Port       GPIOA
#define SW2_Pin             GPIO_PIN_12
#define SW2_GPIO_Port       GPIOA
#define SW3_Pin             GPIO_PIN_6
#define SW3_GPIO_Port       GPIOF
#define SW4_Pin             GPIO_PIN_7
#define SW4_GPIO_Port       GPIOF
#define ST4_DECP_Pin        GPIO_PIN_3
#define ST4_DECP_GPIO_Port  GPIOB
#define ST4_RAP_Pin         GPIO_PIN_4
#define ST4_RAP_GPIO_Port   GPIOB
#define ST4_DECM_Pin        GPIO_PIN_8
#define ST4_DECM_GPIO_Port  GPIOB
#define ST4_RAM_Pin         GPIO_PIN_9
#define ST4_RAM_GPIO_Port   GPIOB

#define RADEC_CLK           PA8
#define RADEC_EN            PA4

#define RA_CSN              PB0
#define RA_SCK              PA5
#define RA_MISO             PA6
#define RA_MOSI             PA7
#define RA_MODE             PC5
#define RA_DIR              PC4
#define RA_STEP             PB10
#define RA_DIAG0            PF5
#define RA_DIAG1            PF4
#define RA_HOME             PC10
#define RA_WORM             PC11
#define RA_GEAR             PC12
#define RA_ENC1             PC3
#define RA_ENC4             PC2

#define HiResEnc2Y          PB5
#define HiResEnc3Y          PA1
#define HiResEnc4Y          PA0

#define DEC_CSN             PB12
#define DEC_SCK             PB13
#define DEC_MISO            PB14
#define DEC_MOSI            PB15
#define DEC_MODE            PB2
#define DEC_DIR             PC7
#define DEC_STEP            PB11
#define DEC_DIAG0           PC8
#define DEC_DIAG1           PC9
#define DEC_HOME            PC15
#define DEC_WORM            PC13
#define DEC_GEAR            PC14
#define DEC_ENC1            PC1
#define DEC_ENC4            PC0

#define V12SENSE            PB1
#define VMBOOST             PA10

#define LED_POLAR           PA9
#define LED_POWER           PA15

#define SW1_GPIO            PA11
#define SW2_GPIO            PA12
#define SW3_GPIO            PF6
#define SW4_GPIO            PF7

#define ST4_DECP            PB3
#define ST4_RAP             PB4
#define ST4_DECM            PB8
#define ST4_RAM             PB9

#define SNAP                PD2

#elif defined(ESP32)
// SPI Pins
#define RA_CSN      33
#define DEC_CSN     33
#define RADEC_EN    13      // DRV_ENN pin in SPI mode

#define SPI_MOSI    26
#define SPI_MISO    32
#define SPI_SCK     25

#define SPI_CS      33      // CS pin in SPI mode
#define SPI_DRV_ENN 13      // DRV_ENN pin in SPI mode

#endif