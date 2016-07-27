

/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file hwtest.c
 *
 * Simple output test.
 * @ref Documentation https://pixhawk.org/dev/examples/write_output
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_config.h>
#include <../modules/uORB/uORB.h>
#include <../modules/uORB/topics/rc_channels.h>

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/actuator_controls.h>
//#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_armed.h>
//#include <uORB/topics/actuator_controls_1.h>
//#include <uORB/topics/actuator_controls_2.h>
//#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_armed.h>
#include <../modules/uORB/topics/rc_channels.h>


#define Degrees(x) ((x) * 57.295779513082321f)

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
    FILE *f_p; //, *f_pr, *f_pc, *f_r, *f_rr, *f_rc,*f_y, *f_yr, *f_yc, *T_p, *T_r;
//    f_r = fopen("/fs/microsd/etc/Roll.txt", "w");
//    f_rr = fopen("/fs/microsd/etc/RollRate.txt", "w");
//    f_rc = fopen("/fs/microsd/etc/RollC.txt", "w");
    f_p = fopen("/fs/microsd/etc/Pitch.txt", "w");
//    f_pr = fopen("/fs/microsd/etc/PitchRate.txt", "w");
//    f_pc = fopen("/fs/microsd/etc/PitchControl.txt", "w");
//    f_y = fopen("/fs/microsd/etc/Yaw.txt", "w");
//    f_yr = fopen("/fs/microsd/etc/YawRate.txt", "w");
//    f_yc = fopen("/fs/microsd/etc/YawC.txt", "w");
//    T_p = fopen("/fs/microsd/etc/TransmitterValuesP.txt", "w");
//    T_r = fopen("/fs/microsd/etc/TransmitterValuesR.txt", "w");

    //declare variables
    double e_roll = 0,e_pitch = 0,e_yaw = 0;
    double sp_roll = 0,sp_pitch = 0, sp_yaw = 0;
    double e_rspeed = 0,e_pspeed = 0,e_yspeed = 0;
    double sp_rspeed = 0, sp_pspeed = 0, sp_yspeed = 0;
    //double saturation = 2;

    double     m_roll, m_pitch, m_yaw, m_pr, m_rr, m_yr;

    // CH3= 0, Newsp_p=0, CH4= 0, Newsp_r=0;
    double Th, Thro;
    int        _att_sub;


    //control and pid values
    double u_r=0,u_p=0,u_y=0;
    double p_r=.2,d_r=0.04;
    double p_p=.35,d_p=.04;
    double p_y=.3,d_y=0;

    //structures
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    orb_advertise(ORB_ID(vehicle_attitude), &att);

    int sensor_sub_fd = orb_subscribe(ORB_ID(rc_channels));
    orb_set_interval(sensor_sub_fd, 1000);

    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));
    orb_advert_t actuator_pub_ptr = orb_advertise(ORB_ID(actuator_controls_0), &actuators);

    //subscribe to attitude data
    _att_sub = orb_subscribe(ORB_ID(vehicle_attitude));


    for(int i=0;i<20000;i++)
    {

    /* get a local copy of attitude */
    /* obtained data for the first file descriptor */
    struct vehicle_attitude_s _att;
    /* copy vehicle_attitude data into local buffer */
    orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

    /* obtained data for the first file descriptor */
    struct rc_channels_s raw;
    /* copy sensors raw data into local buffer */
    orb_copy(ORB_ID(rc_channels), sensor_sub_fd, &raw);

    m_roll = Degrees(_att.roll);
    m_pitch = Degrees(_att.pitch);
    m_yaw = Degrees(_att.yaw);

//    CH3 = (double)raw.channels[1];
//    Newsp_p = 25*CH3;
//
//    CH4 = (double)raw.channels[0];
//    Newsp_r = 25*CH4;

    Th = (double)raw.channels[2];
    Thro = Th;


    m_pr = Degrees (_att.pitchspeed);
    m_rr = Degrees (_att.rollspeed);
    m_yr = Degrees (_att.yawspeed);

//    sp_roll = Newsp_r;
//    sp_pitch = Newsp_p;

    if(i<100)
    {
            sp_yaw = m_yr;
            sp_roll = m_rr;
            sp_pitch = m_pr;
    }

//    if (m_yr <= 360 && m_yr >= 180)
//    {
//
//        m_yr = m_yr - 360;
//    }
//    else
//    {
//        m_yr = m_yr;
//    }

    sp_rspeed = 0;
    sp_pspeed = 0;
    sp_yspeed = 0;

    //find error values
    e_roll = m_roll - sp_roll;
    e_pitch = m_pitch - sp_pitch;
    e_yaw = m_yaw - sp_yaw;

    e_rspeed = m_rr - sp_rspeed;
    e_pspeed = m_pr - sp_pspeed;
    e_yspeed = m_yr - sp_yspeed;

    //find control input
    u_r = e_roll*p_r + e_rspeed*d_r;
    u_p = e_pitch*p_p + e_pspeed*d_p;
    u_y = e_yaw*p_y + e_yspeed*d_y;

    //scale control values to saturation value
    u_r = u_r/90;
    u_p = u_p/90;
    //u_y = 2 * (u_y + saturation)/(saturation*2) - 1;
    u_y = u_y/90;

    //saturate out vales that are larger
    if(u_r>1.0)
    {
        u_r = 1;
    }
    if(u_r<-1.0)
    {
        u_r = -1;
    }
    if(u_p>1.0)
    {
        u_p = 1;
    }
    if(u_p<-1.0)
    {
        u_p = -1;
    }
    if(u_y>1.0)
    {
        u_y = 1;
    }
    if(u_y<-1.0)
    {
        u_y = -1;
    }
//    Roll
//    actuators.control[0] = -.44+u_r-u_y;
//    actuators.control[1] = .1+-2*u_r-u_y;

    actuators.control[0] = Thro;//u_p-u_r;
    actuators.control[1] = Thro;//u_p-u_r;

// Pitch
//    actuators.control[2] = -.6+u_p+u_y; // -u_y
//    actuators.control[3] = -.6-u_p+u_y;

    actuators.control[2] = Thro;//u_p+u_r;
    actuators.control[3] = Thro;//u_p+u_r;

// Rotate Servo as Pixhawk Pitches down.
    if(m_pitch < 65)
    {
        actuators.control[4] = m_pitch/65;
    }

    printf("[rc_channels_test] RC Channels 1-4:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",
                        m_pitch,
                        (double)raw.channels[1],
                        (double)raw.channels[2],
                        (double)raw.channels[3]);

    fprintf(f_p,"%3.3f \n", 10.1);
//    fprintf(f_pr,"%3.3f \n", m_pr);
//    fprintf(f_pc,"%3.3f \n", u_p);
//    fprintf(T_p,"%3.3f \n", CH3);
//    fprintf(T_r,"%3.3f \n", CH4);
//    fprintf(f_r, "%3.3f \n", m_roll);
//    fprintf(f_rr,"%3.3f \n", m_rr);
//    fprintf(f_rc,"%3.3f \n", u_r);
//    fprintf(f_y,"%3.3f \n", m_yaw);
//    fprintf(f_yr,"%3.3f \n", m_yr);
//    fprintf(f_yc,"%3.3f \n", u_y);

// publish control value
    orb_publish(ORB_ID(actuator_controls_0), actuator_pub_ptr, &actuators);

    usleep(2857);

    }

    fclose(f_p);
//    fclose(f_r);
//    fclose(f_pr);
//    fclose(f_pc);
//    fclose(T_p);
//    fclose(T_r);
//    fclose(f_rr);
//    fclose(f_rc);
//    fclose(f_y);
//    fclose(f_yr);
//    fclose(f_yc);

    return OK;
}


