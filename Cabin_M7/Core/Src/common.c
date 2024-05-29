/*
 * common.c
 *
 *  Created on: Oct 24, 2023
 *      Author: ADMIN
 */


#include "common.h"

uint32_t tickDiff(uint32_t startTime, uint32_t currentTime)
{
  if (currentTime < startTime)
  {
    return (INT32_MAX - startTime + currentTime);
  }
  return currentTime - startTime;
}
