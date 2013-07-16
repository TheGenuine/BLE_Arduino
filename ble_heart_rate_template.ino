#include <SPI.h>
#include <avr/pgmspace.h>
#include <ble_system.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "services.h"
#include "heart_rate.h"


 #define O0 11
 #define O1 10
 #define O2 9
 #define O3 6
 #define O4 5
 #define O5 3
 #define I0 A0
 #define I1 A1
 #define I2 A2
 #define I3 A3
 #define I4 A4
 #define I5 A5

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

#define BUFFER_SIZE 20

/*
Store the nRF8001 setup information generated on the flash of the AVR.
This reduces the RAM requirements for the nRF8001.
*/
static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;
static aci_state_t aci_state;

static hal_aci_evt_t aci_data;
static hal_aci_data_t aci_cmd;

static bool radio_ack_pending  = false;
static bool timing_change_done = false;

const int analogInPin = I0; 

uint8_t buffer[BUFFER_SIZE];
short currentPos = 0;
int sensorValue = 60;

void setup(void)
{ 
  Serial.begin(115200);
  Serial.println("Arduino setup");
  
  /**
  Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  */ 
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  /** We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
   *  and initialize the data structures required to setup the nRF8001
   */
  lib_aci_init(&aci_state);
  heart_rate_init();
}

void aci_loop()
{
  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    
    aci_evt = &aci_data.evt;    
   
    switch(aci_evt->evt_opcode)
    {
        case ACI_EVT_DEVICE_STARTED:
        {          
          aci_state.data_credit_available = aci_evt->params.device_started.credit_available;
          switch(aci_evt->params.device_started.device_mode)
          {
            case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            aci_state.device_state = ACI_DEVICE_SETUP;
            Serial.println("Evt Device Started: Setup");
            if (ACI_STATUS_TRANSACTION_COMPLETE != do_aci_setup(&aci_state))
            {
              Serial.println("Error in ACI Setup");
            }
            break;
            
            case ACI_DEVICE_STANDBY:
              aci_state.device_state = ACI_DEVICE_STANDBY;
              Serial.println("Evt Device Started: Standby");
              lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
              Serial.println("Advertising started for 180s");
              break;
          }
        }
        break; //ACI Device Started Event
        
      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status )
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command         

          Serial.print("ACI Status of ACI Evt Cmd Rsp 0x");
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);           
          Serial.print("ACI Command 0x");
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);          
          Serial.println("Evt Cmd respone: Error. Arduino is in an while(1); loop");
          while (1);
        }        
        break;
        
      case ACI_EVT_PIPE_STATUS:
        Serial.println("Evt Pipe Status");
        /** check if the peer has subscribed to the Heart Rate Measurement Characteristic for Notifications
        */
        if (lib_aci_is_pipe_available(&aci_state, PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX) 
            && (false == timing_change_done) )
        {
          /*
          Request a change to the link timing as set in the GAP -> Preferred Peripheral Connection Parameters
          Change the setting in nRFgo studio -> nRF8001 configuration -> GAP Settings and recompile the xml file.
          */
          lib_aci_change_timing_GAP_PPCP();
          timing_change_done = true;
        }      
        break;
        
      case ACI_EVT_TIMING:
        /*
        Link timing has changed.
        */
        Serial.println("Timing changed");
        Serial.println(aci_evt->params.timing.conn_rf_interval, HEX);
        break;
        
      case ACI_EVT_CONNECTED:
        radio_ack_pending  = false;
        timing_change_done = false;
        Serial.println("Evt Connected");
        break;
        
        
      case ACI_EVT_DATA_CREDIT:
        /**
        Bluetooth Radio ack received from the peer radio for the data packet sent.
        This also signals that the buffer used by the nRF8001 for the data packet is available again.
        */
        radio_ack_pending = false;
        break;
      
      case ACI_EVT_PIPE_ERROR:
        /**
        Send data failed. ACI_EVT_DATA_CREDIT will not come.
        This can happen if the pipe becomes unavailable by the peer unsubscribing to the Heart Rate 
        Measurement characteristic.
        This can also happen when the link is disconnected after the data packet has been sent.
        */
        radio_ack_pending = false;
        break;
 
       case ACI_EVT_DISCONNECTED:       
        /**
        Advertise again if the advertising timed out.
        */
        if(ACI_STATUS_ERROR_ADVT_TIMEOUT == aci_evt->params.disconnected.aci_status)
        {
          Serial.println("Evt Disconnected -> Advertising timed out");          
          {
            Serial.println("nRF8001 going to sleep");
            lib_aci_sleep();
            aci_state.device_state = ACI_DEVICE_SLEEP;
          }
        }
        else
        {          
          Serial.println("Evt Disconnected -> Link lost.");
          lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
          Serial.println("Advertising started for 180s");
        }
        break;
       
        
        
    }
  }
  else
  {
    // If No event in the ACI Event queue and No event in the ACI Command queue
    // Arduino can go to sleep
  }
}

/*
Use this function to reset the expended energy iff the Heart Rate Control Point is present.
*/
#ifdef PIPE_HEART_RATE_HEART_RATE_CONTROL_POINT_RX_ACK
void hook_for_resetting_energy_expended(void)
{
}
#endif

boolean isBufferFull() {
  return currentPos == 19;
  // return currentPos >= (buffer_size / 2);
}

boolean isChannelOpen() {
  return lib_aci_is_pipe_available(&aci_state, PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX) && (radio_ack_pending == false) && (timing_change_done == true);
}

void loop()
{
  // sensorValue = analogRead(analogInPin);
  sensorValue++;

  Serial.print(sensorValue);
  
  buffer[currentPos] = sensorValue;
  currentPos++;

  Serial.print(" @ ");
  Serial.println(currentPos);

  aci_loop();
  
  if (isBufferFull() && isChannelOpen())
  {
      heart_rate_set_support_contact_bit();
      heart_rate_set_contact_status_bit();

     
        Serial.print("Send Data - ");
        boolean success = heart_rate_send_hr_bulk(buffer, currentPos);
        
        Serial.println(success);
        radio_ack_pending = true;
  }

  if(currentPos == BUFFER_SIZE) {
    currentPos = 0;
  } 
  delay(200);
}
