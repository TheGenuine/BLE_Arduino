/* Copyright (c) 2011 Nordic Semiconductor. All Rights Reserved.
*
* The information contained herein is property of Nordic Semiconductor ASA.
* Terms and conditions of usage are described in detail in NORDIC
* SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
*
* Licensees are granted free, non-transferable use of the information. NO
* WARRANTY of ANY KIND is provided. This heading must NOT be removed from
* the file.
*
* $LastChangedRevision$
*/

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include "lib_aci.h"
#include "services.h"
#include "heart_rate.h"



static uint8_t current_heart_rate_data[HR_MAX_PAYLOAD];

void heart_rate_init()
{
  uint8_t i;
  for (i = 0; i < HR_MAX_PAYLOAD; i ++)
  {
    current_heart_rate_data[i] = 0;
  }
}

void heart_rate_set_support_contact_bit()
{
  current_heart_rate_data[0] |= HEART_RATE_FLAGS_T_SENSOR_CONTACT_SUPPORT;
}

void heart_rate_clear_support_contact_bit()
{
  current_heart_rate_data[0] &= ~HEART_RATE_FLAGS_T_SENSOR_CONTACT_SUPPORT;
}

void heart_rate_set_contact_status_bit()
{
  current_heart_rate_data[0] |= HEART_RATE_FLAGS_T_SENSOR_CONTACT_STATUS;
}

void heart_rate_clear_contact_status_bit()
{
  current_heart_rate_data[0] &= ~HEART_RATE_FLAGS_T_SENSOR_CONTACT_STATUS;
}

bool  heart_rate_send_hr_bulk(int meas_hr[], short index) {
  uint8_t data_index = 0;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  data_index++;
  while(data_index < HR_MAX_PAYLOAD || data_index < index) {
    current_heart_rate_data[data_index] = meas_hr[data_index -1];
    data_index++;
  }
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                                 (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

bool heart_rate_send_hr(uint8_t meas_hr)
{
  uint8_t data_index = 0;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  data_index++;
  current_heart_rate_data[data_index++] = meas_hr;
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                           (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

bool heart_rate_send_hr_16bits(uint16_t meas_hr)
{
  uint8_t data_index = 0;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  data_index++;
  current_heart_rate_data[data_index++] = (uint8_t)meas_hr;
  current_heart_rate_data[data_index++] = (uint8_t)(meas_hr>>8);
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                           (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

bool heart_rate_send_hr_expended_energy(uint8_t meas_hr, uint16_t expended_energy)
{
  uint8_t data_index = 0;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  data_index++;
  current_heart_rate_data[data_index++] = meas_hr;
  current_heart_rate_data[data_index++] = (uint8_t)expended_energy;
  current_heart_rate_data[data_index++] = (uint8_t)(expended_energy>>8);
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                           (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

bool heart_rate_send_hr_16bits_expended_energy(uint16_t meas_hr, uint16_t expended_energy)
{
  uint8_t data_index = 0;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  data_index++;
  current_heart_rate_data[data_index++] = (uint8_t)meas_hr;
  current_heart_rate_data[data_index++] = (uint8_t)(meas_hr>>8);
  current_heart_rate_data[data_index++] = (uint8_t)expended_energy;
  current_heart_rate_data[data_index++] = (uint8_t)(expended_energy>>8);
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                           (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

bool heart_rate_send_hr_rr_interval(uint8_t meas_hr, uint16_t *p_rr_intervals, uint8_t nb_intervals)
{
  uint8_t data_index = 0, i;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  data_index++;
  current_heart_rate_data[data_index++] = meas_hr;
  for(i = 0; i < nb_intervals; i ++)
  {
    current_heart_rate_data[data_index++] = (uint8_t) (p_rr_intervals[i]);
    current_heart_rate_data[data_index++] = (uint8_t)((p_rr_intervals[i])>>8);
  }
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                           (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

bool heart_rate_send_hr_16bits_rr_interval(uint16_t meas_hr, uint16_t *p_rr_intervals, uint8_t nb_intervals)
{
  uint8_t data_index = 0, i;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  data_index++;
  current_heart_rate_data[data_index++] = (uint8_t)meas_hr;
  current_heart_rate_data[data_index++] = (uint8_t)(meas_hr>>8);
  for(i = 0; i < nb_intervals; i ++)
  {
    current_heart_rate_data[data_index++] = (uint8_t) (p_rr_intervals[i]);
    current_heart_rate_data[data_index++] = (uint8_t)((p_rr_intervals[i])>>8);
  }
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                           (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

bool heart_rate_send_hr_expended_energy_rr_interval(uint8_t meas_hr, uint16_t expended_energy, uint16_t *p_rr_intervals, uint8_t nb_intervals)
{
  uint8_t data_index = 0, i;
  current_heart_rate_data[data_index] &= ~HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  data_index++;
  current_heart_rate_data[data_index++] = meas_hr;
  current_heart_rate_data[data_index++] = (uint8_t)expended_energy;
  current_heart_rate_data[data_index++] = (uint8_t)(expended_energy>>8);
  for(i = 0; i < nb_intervals; i ++)
  {
    current_heart_rate_data[data_index++] = (uint8_t) (p_rr_intervals[i]);
    current_heart_rate_data[data_index++] = (uint8_t)((p_rr_intervals[i])>>8);
  }
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                           (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

bool heart_rate_send_hr_16bits_expended_energy_rr_interval(uint16_t meas_hr, uint16_t expended_energy, uint16_t *p_rr_intervals, uint8_t nb_intervals)
{
  uint8_t data_index = 0, i;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_MEAS_SIZE_BIT;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_ENERGY_EXPENDED_STATUS_BIT;
  current_heart_rate_data[data_index] |= HEART_RATE_FLAGS_RR_INTERVAL_SUPPORT_BIT;
  data_index++;
  current_heart_rate_data[data_index++] = (uint8_t)meas_hr;
  current_heart_rate_data[data_index++] = (uint8_t)(meas_hr>>8);
  current_heart_rate_data[data_index++] = (uint8_t)expended_energy;
  current_heart_rate_data[data_index++] = (uint8_t)(expended_energy>>8);
  for(i = 0; i < nb_intervals; i ++)
  {
    current_heart_rate_data[data_index++] = (uint8_t) (p_rr_intervals[i]);
    current_heart_rate_data[data_index++] = (uint8_t)((p_rr_intervals[i])>>8);
  }
  return lib_aci_send_data(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX,
                           (uint8_t *)&current_heart_rate_data[0] ,data_index);
}

#ifdef PIPE_HEART_RATE_HEART_RATE_CONTROL_POINT_RX_ACK
void heart_rate_pipes_updated_evt_rcvd(aci_state_t *aci_stat, uint8_t pipe_num, uint8_t *buffer)
{
  switch (pipe_num)
  {
    case PIPE_HEART_RATE_HEART_RATE_CONTROL_POINT_RX_ACK :
      if (buffer[0] == HRCP_OPCODE_RESET_ENERY_EXPENDED)
      {
        lib_aci_send_ack(aci_stat, PIPE_HEART_RATE_HEART_RATE_CONTROL_POINT_RX_ACK);
        hook_for_resetting_energy_expended();
      }
      else
      {
        /* Value received is reserved for future use. Send error response*/
        lib_aci_send_nack(aci_stat, PIPE_HEART_RATE_HEART_RATE_CONTROL_POINT_RX_ACK, HRCP_ERR_CONTROL_POINT_NOT_SUPPORTED);
      }
      break;
    default:
      /* Do nothing*/
      break;
  }
}
#endif
