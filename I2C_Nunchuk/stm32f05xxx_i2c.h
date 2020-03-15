#ifndef __STM32F05XXX_I2C_H
#define __STM32F05XXX_I2C_H

#define 	I2C_CR1_PECEN   (1 << 23)
#define 	I2C_CR1_ALERTEN   (1 << 22)
#define 	I2C_CR1_SMBDEN   (1 << 21)
#define 	I2C_CR1_SMBHEN   (1 << 20)
#define 	I2C_CR1_GCEN   (1 << 19)
#define 	I2C_CR1_WUPEN   (1 << 18)
#define 	I2C_CR1_NOSTRETCH   (1 << 17)
#define 	I2C_CR1_SBC   (1 << 16)
#define 	I2C_CR1_RXDMAEN   (1 << 15)
#define 	I2C_CR1_TXDMAEN   (1 << 14)
#define 	I2C_CR1_ANFOFF   (1 << 12)
#define 	I2C_CR1_DNF_MASK   0xF
#define 	I2C_CR1_DNF_SHIFT   8
#define 	I2C_CR1_ERRIE   (1 << 7)
#define 	I2C_CR1_TCIE   (1 << 6)
#define 	I2C_CR1_STOPIE   (1 << 5)
#define 	I2C_CR1_NACKIE   (1 << 4)
#define 	I2C_CR1_ADDRIE   (1 << 3)
#define 	I2C_CR1_RXIE   (1 << 2)
#define 	I2C_CR1_TXIE   (1 << 1)
#define 	I2C_CR1_PE   (1 << 0)
 
#define 	I2C_CR2_PECBYTE   (1 << 26)
#define 	I2C_CR2_AUTOEND   (1 << 25)
#define 	I2C_CR2_RELOAD   (1 << 24)
#define 	I2C_CR2_NBYTES_SHIFT   16
#define 	I2C_CR2_NBYTES_MASK   (0xFF << I2C_CR2_NBYTES_SHIFT)
#define 	I2C_CR2_NACK   (1 << 15)
#define 	I2C_CR2_STOP   (1 << 14)
#define 	I2C_CR2_START   (1 << 13)
#define 	I2C_CR2_HEAD10R   (1 << 12)
#define 	I2C_CR2_ADD10   (1 << 11)
#define 	I2C_CR2_RD_WRN   (1 << 10)
#define 	I2C_CR2_SADD_7BIT_SHIFT   1
#define 	I2C_CR2_SADD_10BIT_SHIFT   0
#define 	I2C_CR2_SADD_7BIT_MASK   (0x7F << I2C_CR2_SADD_7BIT_SHIFT)
#define 	I2C_CR2_SADD_10BIT_MASK   0x3FF

#define 	I2C_OAR1_OA1EN_DISABLE   (0x0 << 15)
#define 	I2C_OAR1_OA1EN_ENABLE   (0x1 << 15)
#define 	I2C_OAR1_OA1MODE   (1 << 10)
#define 	I2C_OAR1_OA1MODE_7BIT   0
#define 	I2C_OAR1_OA1MODE_10BIT   1
#define 	I2C_OAR1_OA1   (1 << 10)
#define 	I2C_OAR1_OA1_7BIT   0
#define 	I2C_OAR1_OA1_10BIT   1
#define 	I2C_OAR2_OA2EN   (1 << 15)
#define 	I2C_OAR2_OA2MSK_NO_MASK   (0x0 << 8)
#define 	I2C_OAR2_OA2MSK_OA2_7_OA2_2   (0x1 << 8)
#define 	I2C_OAR2_OA2MSK_OA2_7_OA2_3   (0x2 << 8)
#define 	I2C_OAR2_OA2MSK_OA2_7_OA2_4   (0x3 << 8)
#define 	I2C_OAR2_OA2MSK_OA2_7_OA2_5   (0x4 << 8)
#define 	I2C_OAR2_OA2MSK_OA2_7_OA2_6   (0x5 << 8)
#define 	I2C_OAR2_OA2MSK_OA2_7   (0x6 << 8)
#define 	I2C_OAR2_OA2MSK_NO_CMP   (0x7 << 8)
 
#define 	I2C_TIMINGR_PRESC_SHIFT   28
#define 	I2C_TIMINGR_PRESC_MASK   (0xF << 28)
#define 	I2C_TIMINGR_SCLDEL_SHIFT   20
#define 	I2C_TIMINGR_SCLDEL_MASK   (0xF << I2C_TIMINGR_SCLDEL_SHIFT)
#define 	I2C_TIMINGR_SDADEL_SHIFT   16
#define 	I2C_TIMINGR_SDADEL_MASK   (0xF << I2C_TIMINGR_SDADEL_SHIFT)
#define 	I2C_TIMINGR_SCLH_SHIFT   8
#define 	I2C_TIMINGR_SCLH_MASK   (0xFF << I2C_TIMINGR_SCLH_SHIFT)
#define 	I2C_TIMINGR_SCLL_SHIFT   0
#define 	I2C_TIMINGR_SCLL_MASK   (0xFF << I2C_TIMINGR_SCLL_SHIFT)
#define 	I2C_TIEMOUTR_TEXTEN   (1 << 31)
#define 	I2C_TIEMOUTR_TIMOUTEN   (1 << 15)
#define 	I2C_TIEMOUTR_TIDLE_SCL_LOW   (0x0 << 12)
#define 	I2C_TIEMOUTR_TIDLE_SCL_SDA_HIGH   (0x1 << 12)
 
#define 	I2C_ISR_DIR_READ   (0x1 << 16)
#define 	I2C_ISR_DIR_WRITE   (0x0 << 16)
#define 	I2C_ISR_BUSY   (1 << 15)
#define 	I2C_ISR_ALERT   (1 << 13)
#define 	I2C_ISR_TIMEOUT   (1 << 12)
#define 	I2C_ISR_PECERR   (1 << 11)
#define 	I2C_ISR_OVR   (1 << 10)
#define 	I2C_ISR_ARLO   (1 << 9)
#define 	I2C_ISR_BERR   (1 << 8)
#define 	I2C_ISR_TCR   (1 << 7)
#define 	I2C_ISR_TC   (1 << 6)
#define 	I2C_ISR_STOPF   (1 << 5)
#define 	I2C_ISR_NACKF   (1 << 4)
#define 	I2C_ISR_ADDR   (1 << 3)
#define 	I2C_ISR_RXNE   (1 << 2)
#define 	I2C_ISR_TXIS   (1 << 1) 
#define 	I2C_ISR_TXE   (1 << 0)
 
#define 	I2C_ICR_ALERTCF   (1 << 13)
#define 	I2C_ICR_TIMOUTCF   (1 << 12)
#define 	I2C_ICR_PECCF   (1 << 11)
#define 	I2C_ICR_OVRCF   (1 << 10)
#define 	I2C_ICR_ARLOCF   (1 << 9)
#define 	I2C_ICR_BERRCF   (1 << 8)
#define 	I2C_ICR_STOPCF   (1 << 5)
#define 	I2C_ICR_NACKCF   (1 << 4)
#define 	I2C_ICR_ADDRCF   (1 << 3)
#endif  // __STM32F05XXX_I2C_H                           
