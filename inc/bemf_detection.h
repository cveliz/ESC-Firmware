/*
 * bemf_detection.h
 *
 *  Created on: 12/02/2018
 *      Author: chepe
 */

#ifndef BEMF_DETECTION_H_
#define BEMF_DETECTION_H_

void adcInit(void);
void dmaInit(void);
void dmaTransferConfig(uint32_t addrs, uint16_t len);

#endif /* BEMF_DETECTION_H_ */
