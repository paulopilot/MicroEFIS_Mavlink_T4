/***************************************************
  Name
    MavlinkFunctions.h
    
  Description
    Mavilink protocol functions

  Author
    Paulo Almeida
    falmeida.paulo@gmail.com
    https://www.youtube.com/channel/UC-kUryZXKOTKsRH2tHRS1NA

  Updates
 ****************************************************/
 
#include "mavlink/1.0/ardupilotmega/mavlink.h"
#include "mavlink/1.0/common/common.h"

// Initialize the required buffers 
mavlink_set_mode_t        mode;
mavlink_system_t          mavlink_system;
mavlink_message_t         heartbeatMsg;
mavlink_heartbeat_t       heartbeat;

// Initialize the required variables
uint32_t heartbeatTimer_TX        = millis();
uint32_t heartbeatTimer_RX        = millis();
uint32_t heartbeatInterval_TX     =  1.0L * 1000L;
uint32_t heartbeatInterval_RX     = 10.0L * 1000L;
bool     mavLink_Connected        = false;

uint8_t bufTx[MAVLINK_MAX_PACKET_LEN];
uint8_t bufRx[MAVLINK_MAX_PACKET_LEN];
uint8_t system_type = MAV_TYPE_FIXED_WING; //MAV_TYPE_GROUND_ROVER; //MAV_TYPE_HELICOPTER;//MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC; //MAV_AUTOPILOT_ARDUPILOTMEGA

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

/***************************************************************************************
** Function name:           clearVariables()
** Description:             Initialize all valiable used from mavlink
***************************************************************************************/
void clearVariables() {
  apm.connected           = false;
  apm.hb_count            = 0;
  apm.vbatA               = 0;
  apm.ibatA               = 0;
  apm.remainingA          = 0;
  //apm.motor_armed         = false;
  //apm.position.lat        = 0;
  //apm.position.lon        = 0;
  //apm.position.alt        = 0;
  apm.fix_type            = GPS_FIX_TYPE_NO_GPS;
  apm.satellites_visible  = 0;
  apm.airspeed            = 0;
  apm.groundspeed         = 0;
  apm.heading             = 0;
  apm.throttle            = 0;
  apm.alt                 = 0;
  apm.climb               = 0;
  apm.ahrs.pitch          = 0;
  apm.ahrs.roll           = 0;
  apm.ahrs.yaw            = 0;
  apm.ahrs.sinRoll        = 0;
  apm.ahrs.cosRoll        = 0;
  flight_mode             = -1;
  wind.direction          = 0;
  wind.speed              = 0;
  wind.speed_z            = 0;
}

/***************************************************************************************
** Function name:           request_mavlink_rates()
** Description:             
***************************************************************************************/
void request_mavlink_rates()
{
  const int  maxStreams = 6;
  const uint8_t MAVStreams[maxStreams] = {
      MAV_DATA_STREAM_RAW_SENSORS,
      MAV_DATA_STREAM_EXTENDED_STATUS,
      MAV_DATA_STREAM_RC_CHANNELS,
      MAV_DATA_STREAM_POSITION,
      MAV_DATA_STREAM_EXTRA1, 
      MAV_DATA_STREAM_EXTRA2};
  const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};
  for (int i=0; i < maxStreams; i++) {
    //mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,apm_mav_system, apm_mav_component, MAVStreams[i], MAVRates[i], 1);
  }
}

/***************************************************************************************
** Function name:           handle_Messages()
** Description:             handle all mavlink massages
***************************************************************************************/
void handle_Messages(){
  mavlink_message_t         receivedMsg;
  mavlink_status_t          _status;

  static long new_ahrs = millis();
  static long max_ahrs = 0;

  if((((millis() - heartbeatTimer_RX) > heartbeatInterval_RX)) && apm.connected) { // if no HEARTBEAT from APM in 5s then we are not connected
    clearVariables();
  } 
    
  while(telem.available()) {
    
    uint8_t c = telem.read();
  
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &receivedMsg, &_status)) {
        //print_heartbeat();
        //if(receivedMsg.msgid > 0){
          //debug.print(" -> Msg ID: ");
          //debug.println(receivedMsg.msgid, DEC);
        //}

        //debug.printf("Received message with ID %d, sequence: %d from component %d of system %d\n", receivedMsg.msgid, receivedMsg.seq, receivedMsg.compid, receivedMsg.sysid);

        switch(receivedMsg.msgid)
        {
          case MAVLINK_MSG_ID_HEARTBEAT:{      // MAV ID: 0
                mavbeat = 1;
                mavlink_heartbeat_t packet;
                mavlink_msg_heartbeat_decode(&receivedMsg, &packet);

                mode.custom_mode = packet.custom_mode;
                flight_mode = mode.custom_mode;
                apm_mav_type = packet.type;
                //packet.autopilot;
                mode.base_mode = packet.base_mode;
                //packet.system_status;
                //packet.mavlink_version;
                
                apm_mav_system    = receivedMsg.sysid;
                apm_mav_component = receivedMsg.compid;
                //apm_mav_type      = mavlink_msg_heartbeat_get_type(&receivedMsg);            
                //flight_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&receivedMsg);
                
                mode.base_mode = mavlink_msg_heartbeat_get_base_mode(&receivedMsg);
                if(bitRead(mode.base_mode,7)) 
                  apm.motor_armed = true;
                else 
                  apm.motor_armed = false;

                osd_nav_mode = 0;          
                //mode.base_mode = (mavlink_msg_heartbeat_get_base_mode(&receivedMsg) & 0x80) > 7;
                heartbeatTimer_RX = millis(); //Reset receive timer
                if(!apm.connected); {
                  apm.hb_count++; 
                  if((apm.hb_count++) > 10) { // If received > 10 heartbeats from MavLink then we are connected
                    apm.connected = true;
                    apm.hb_count=0;
                    //digitalWrite(led,HIGH); // LED will be ON when connected to MavLink, else it will slowly blink
                  }
                }
                break;
          }
          case MAVLINK_MSG_ID_SYS_STATUS:{      // MAV ID: 1
                mavlink_sys_status_t packet;
                mavlink_msg_sys_status_decode(&receivedMsg, &packet);

                apm.vbatA = packet.voltage_battery / 1000.0f; //Battery voltage, in millivolts (1 = 1 millivolt)
                apm.ibatA = packet.current_battery * 0.01f;    
                apm.remainingA = packet.battery_remaining; //Remaining battery energy: (0%: 0, 100%: 100)
                //debug.println("MAVLINK_MSG_ID_SYS_STATUS");
                //debug.printf("VbatA: %02.1fV IbatA: %03.1fma Remain:%3d%%\n\n", apm.vbatA, apm.ibatA, (int)apm.remainingA);
                break;
          }
          case MAVLINK_MSG_ID_SYSTEM_TIME:{      // MAV ID: 2
                mavlink_system_time_t packet;
                mavlink_msg_system_time_decode(&receivedMsg, &packet);

                time_boot_ms = packet.time_boot_ms;
                break;
          }
          case MAVLINK_MSG_ID_SET_MODE:{       // MAV ID: 11
                if(mode.base_mode == MAV_MODE_GUIDED_ARMED || mode.base_mode == MAV_MODE_MANUAL_ARMED){
                  debug.println("SYSTEM ARMED");
                  //print_setMode();
                  //motor_control = 1;
                } else {
                  debug.print("--> New Base Mode: ");
                  debug.println(mode.base_mode);
                }
                break;
          }
          case MAVLINK_MSG_ID_GPS_RAW_INT:{      // MAV ID: 24
                // decode
                mavlink_gps_raw_int_t packet;
                mavlink_msg_gps_raw_int_decode(&receivedMsg, &packet);
                
                apm.fix_type = packet.fix_type;
                apm.satellites_visible = packet.satellites_visible;
                apm.time_usec = packet.time_usec;
                //apm.alt = packet.alt;
                break;
          }
          case MAVLINK_MSG_ID_GPS_STATUS : {      // MAV ID: 25
                //debug.println("MAVLINK_MSG_ID_GPS_STATUS");
                break;
          }
          case MAVLINK_MSG_ID_SCALED_PRESSURE:{   // MAV ID: 29
                // decode
                mavlink_scaled_pressure_t packet;
                mavlink_msg_scaled_pressure_decode(&receivedMsg, &packet);
                apm.temperature = (float)(packet.temperature / 100.0);
                break;
          }
          case MAVLINK_MSG_ID_ATTITUDE:{      // MAV ID: 30
                // decode
                mavlink_attitude_t packet;
                mavlink_msg_attitude_decode(&receivedMsg, &packet);

                // save the attitude
                apm.ahrs.roll       = (packet.roll * RAD_TO_DEG);
                apm.ahrs.pitch      = -(packet.pitch * RAD_TO_DEG);
                apm.ahrs.yaw        = packet.yaw * RAD_TO_DEG;
                apm.ahrs.rollspeed  = packet.rollspeed;
                apm.ahrs.pitchspeed = packet.pitchspeed;
                apm.ahrs.yawspeed   = packet.yawspeed;
                apm.ahrs.sinRoll    = sin(apm.ahrs.roll * DEG_TO_RAD);
                apm.ahrs.cosRoll     = cos(apm.ahrs.roll * DEG_TO_RAD);

                /******************************************************
                long intervalo = millis() - new_ahrs;
                if (intervalo > max_ahrs) {
                  max_ahrs = intervalo;
                }
                //debug.print(receivedMsg.msgid, DEC);
                debug.printf(" %d ms -> máx. %d ms\n", int(intervalo), int(max_ahrs) );
                new_ahrs = millis();
                //debug.printf("APM Pitch: %03.0f Roll: %03.0f Yaw: %03.0f\n\n", apm.ahrs.pitch, apm.ahrs.roll, apm.ahrs.yaw);
                /*****************************************************/
                
                //debug.println("MAVLINK_MSG_ID_ATTITUDE");
                break;
          }
          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:{      // MAV ID: 33
                // decode
                mavlink_global_position_int_t packet;
                mavlink_msg_global_position_int_decode(&receivedMsg, &packet);

                // Save the position
                apm.position.lat = packet.lat / 10000000.0; // mavlink sends value * 1E7
                apm.position.lon = packet.lon / 10000000.0;
                //apm.alt = packet.alt / 1000.0; // mavlink sed alt in mm
                apm.alt = packet.relative_alt / 1000.0; // mavlink sed alt in mm
                apm.relative_alt = packet.relative_alt / 1000.0;
                //apm.heading = packet.hdg;   // compass heading
                //uav.vx = packet.vx;
                //uav.vy = packet.vy;
                //uav.vz = packet.vz;
                //uav.vel = sqrt(uav.vx*uav.vx + uav.vy*uav.vy + uav.vz*uav.vz);
                //debug.println("MAVLINK_MSG_ID_GLOBAL_POSITION_INT");
                //debug.printf("Lat:%f Lon:%f Alt:%3.1f RelAlt:%3.1f\n\n", apm.position.lat, apm.position.lon, apm.alt, apm.relative_alt);
                break;
          }
          case MAVLINK_MSG_ID_RC_CHANNELS_RAW:{      // MAV ID: 35
                // decode
                mavlink_rc_channels_raw_t packet;
                mavlink_msg_rc_channels_raw_decode(&receivedMsg, &packet);

                raw_channel[0].value = packet.chan1_raw;
                raw_channel[1].value = packet.chan2_raw;
                raw_channel[2].value = packet.chan3_raw;
                raw_channel[3].value = packet.chan4_raw;
                raw_channel[4].value = packet.chan5_raw;
                raw_channel[5].value = packet.chan6_raw;
                raw_channel[6].value = packet.chan7_raw;
                raw_channel[7].value = packet.chan8_raw;
                
                //debug.println("MAVLINK_MSG_ID_RC_CHANNELS_RAW");
                break;
          }
          case MAVLINK_MSG_ID_MISSION_CURRENT:{      // MAV ID: 42
                wp.wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&receivedMsg);
                //debug.println("MAVLINK_MSG_ID_MISSION_CURRENT");
                //debug.printf("WP: %03d\n\n", (int)wp.wp_number);
                break;
          }
          case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:{      // MAV ID: 62
                // decode
                mavlink_nav_controller_output_t packet;
                mavlink_msg_nav_controller_output_decode(&receivedMsg, &packet);

                wp.ahrs.roll = packet.nav_roll;                /*< [deg] Current desired roll*/
                wp.ahrs.pitch = packet.nav_pitch;              /*< [deg] Current desired pitch*/
                wp.ahrs.yaw = 0;
                wp.alt_error = packet.alt_error;               /*< [m] Current altitude error*/
                wp.aspd_error = packet.aspd_error;             /*< [m/s] Current airspeed error*/
                wp.xtrack_error = packet.xtrack_error;         /*< [m] Current crosstrack error on x-y plane*/
                wp.nav_bearing = packet.nav_bearing;           /*< [deg] Current desired heading*/
                wp.target_bearing = packet.target_bearing;     /*< [deg] Bearing to current waypoint/target*/
                wp.wp_dist = (uint32_t)packet.wp_dist;         /*< [m] Distance to active waypoint*/
                //debug.println("MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT");
                //debug.printf("NAV Pitch:%03.1f Roll:%03.1f AltErr:%03.1f\n", wp.ahrs.pitch, wp.ahrs.roll, wp.alt_error);
                //debug.printf("NAV Dist:%d BRG:%3d\n", (int)wp.wp_dist, (int)wp.nav_bearing);   
                break;
          }
          case MAVLINK_MSG_ID_VFR_HUD:{      // MAV ID: 74
                // decode
                mavlink_vfr_hud_t packet;
                mavlink_msg_vfr_hud_decode(&receivedMsg, &packet);
            
                apm.airspeed = packet.airspeed;                 /*< Current airspeed in m/s*/
                apm.groundspeed = packet.groundspeed;           /*< Current ground speed in m/s*/
                //apm.alt = packet.alt;                           /*< Current altitude (MSL), in meters*/
                apm.climb = packet.climb;                       /*< Current climb rate in meters/second*/
                apm.heading = packet.heading;                   /*< Current heading in degrees, in compass units (0..360, 0=north)*/
                apm.throttle = packet.throttle;                 /*< Current throttle setting in integer percent, 0 to 100*/
                //debug.println("MAVLINK_MSG_ID_VFR_HUD");
                //debug.printf("IAS:%03d GS:%03d HDG:%03d THR:%03d%%\n", (int)apm.airspeed, (int)apm.groundspeed, (int)apm.heading, (int)apm.throttle);
                //debug.printf("ALT:%03.1f CLIMB:%03.1f\n", packet.alt, apm.climb);
                if (apm.heading < 0) apm.heading = 0;
                if (apm.heading > 360) apm.heading = 360;
                break;
          }
          /*
          case MAVLINK_MSG_ID_MANUAL_SETPOINT:{       // MAV ID: 81
                // decode
                mavlink_manual_setpoint_t packet;
                mavlink_msg_manual_setpoint_decode(&receivedMsg, &packet);

                apm.ahrs.target.roll_rate = packet.roll;
                apm.ahrs.target.pitch_rate = packet.pitch;
                apm.ahrs.target.yaw_rate = packet.yaw;
                apm.ahrs.target.thrust = packet.thrust;
                debug.printf("R.Rate:%3.1f P.Rate:%f Y.Rate:%3.1f Thrust:%3.1f\n", apm.ahrs.target.roll_rate, apm.ahrs.target.pitch_rate, apm.ahrs.target.yaw_rate, apm.ahrs.target.thrust);
                break;
          }
          case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:{      // MAV ID: 140
                break;
          }

          case MAVLINK_MSG_ID_FOLLOW_TARGET:{       // MAV ID: 144
                // decode
                mavlink_follow_target_t packet;
                mavlink_msg_follow_target_decode(&receivedMsg, &packet);

                apm.ahrs.target.roll_rate = packet.rates[0];
                apm.ahrs.target.pitch_rate = packet.rates[1];
                apm.ahrs.target.yaw_rate = packet.rates[2];
                //apm.ahrs.target.thrust = packet.thrust;
                debug.printf("R.Rate:%3.1f P.Rate:%f Y.Rate:%3.1f\n", apm.ahrs.target.roll_rate, apm.ahrs.target.pitch_rate, apm.ahrs.target.yaw_rate);
                break;
          }
          */

          case MAVLINK_MSG_ID_WIND:{      // MAV ID: 168
                // decode
                mavlink_wind_t packet;
                mavlink_msg_wind_decode(&receivedMsg, &packet);
                
                wind.direction = packet.direction; // 0..360 deg, 0=north
                wind.speed = packet.speed; // m/s
                wind.speed_z = packet.speed_z; // m/s
                //debug.println("MAVLINK_MSG_ID_WIND");
                break;
          }
          case MAVLINK_MSG_ID_HOME_POSITION:{      // MAV ID: 242
                // decode
                /*
                mavlink_home_position_t packet;
                mavlink_msg_home_position_decode(&receivedMsg, &packet);

                home.position.lat = packet.latitude / 10000000.0; // mavlink sends value * 1E7
                home.position.lon = packet.longitude / 10000000.0; // mavlink sends value * 1E7
                home.position.alt = packet.altitude / 1000.0; // mavlink sed alt in mm
                debug.println("MAVLINK_MSG_ID_HOME_POSITION");
                debug.printf("Lat:%f Lon:%f Alt:%3.1f\n\n", home.position.lat, home.position.lon, home.position.alt);
                */
                break;
          }
          case  MAVLINK_MSG_ID_ADSB_VEHICLE:{ // MAV ID: 246
                break;
          }
          default:
              //Do nothing
                break;
        }
    }
  }
}

/***************************************************************************************
** Function name:           send_heartbeat()
** Description:             prepare and send heart beat to mavlink
***************************************************************************************/
void send_heartbeat(){
    memset(bufTx, 0xFF, sizeof(bufTx));
    mavlink_system.sysid = MAV_TYPE_FIXED_WING ;
    mavlink_system.compid = MAV_AUTOPILOT_GENERIC; 
    
    // Pack the message 
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &heartbeatMsg, system_type, autopilot_type, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
  
    // Copy the message to send buffer 
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &heartbeatMsg);
 
    //Write Message    
    telem.write(bufTx, len);        
    heartbeatTimer_TX = millis();
    debug.println("Heartbeat"); 
}
