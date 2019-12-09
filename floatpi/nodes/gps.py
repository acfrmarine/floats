#!/usr/bin/env python
import navio2.util
import navio2.ublox
from sensor_msgs.msg import NavSatFix
import rospy

if __name__ == "__main__":
    rospy.init_node('navio_gps_node')
    pub = rospy.Publisher('navio/gps', NavSatFix, queue_size=1)

    ubl = navio2.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

    ubl.configure_poll_port()
    ubl.configure_poll(navio2.ublox.CLASS_CFG, navio2.ublox.MSG_CFG_USB)
    #ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW)

    ubl.configure_port(port=navio2.ublox.PORT_SERIAL1, inMask=1, outMask=0)
    ubl.configure_port(port=navio2.ublox.PORT_USB, inMask=1, outMask=1)
    ubl.configure_port(port=navio2.ublox.PORT_SERIAL2, inMask=1, outMask=0)
    ubl.configure_poll_port()
    ubl.configure_poll_port(navio2.ublox.PORT_SERIAL1)
    ubl.configure_poll_port(navio2.ublox.PORT_SERIAL2)
    ubl.configure_poll_port(navio2.ublox.PORT_USB)
    ubl.configure_solution_rate(rate_ms=1000)

    ubl.set_preferred_dynamic_model(None)
    ubl.set_preferred_usePPP(None)

    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_POSLLH, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_PVT, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_STATUS, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_SOL, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_VELNED, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_SVINFO, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_VELECEF, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_POSECEF, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_RAW, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_SFRB, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_SVSI, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_ALM, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_RXM, navio2.ublox.MSG_RXM_EPH, 1)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_TIMEGPS, 5)
    ubl.configure_message_rate(navio2.ublox.CLASS_NAV, navio2.ublox.MSG_NAV_CLOCK, 5)
    #ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_DGPS, 5)

    r = rospy.Rate(10) # 10hz
    count = 0
    while not rospy.is_shutdown():
        msg = ubl.receive_message()
        if msg is None:
            if opts.reopen:
                ubl.close()
                ubl = navio2.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
                continue
            break
        if msg.name() == "NAV_POSLLH":
            msgstrl = str(msg).split(",")[1:]
            longitude = float(msgstrl[0].split('=')[-1])/10000000
            latitude = float(msgstrl[1].split('=')[-1])/10000000
            height = float(msgstrl[2].split('=')[-1])/1000
            hMSL = float(msgstrl[3].split('=')[-1])/1000
            hACC = float(msgstrl[4].split('=')[-1])/1000
            vACC = float(msgstrl[5].split('=')[-1])/1000

            navmsg = NavSatFix()
            navmsg.header.stamp = rospy.Time.now()
            navmsg.header.frame_id = 'gps'
            navmsg.header.seq = count
            count += 1
            navmsg.latitude = latitude
            navmsg.longitude = longitude
            navmsg.altitude = height
            navmsg.position_covariance_type = navmsg.COVARIANCE_TYPE_KNOWN
            navmsg.position_covariance = [hACC, hACC, 0, hACC, hACC, 0, 0, 0, vACC]
            if valid_gps == 0:
                navmsg.status.status = -1
            else:
                navmsg.status.status = 0
            navmsg.status.service = 1
            pub.publish(navmsg)

        if msg.name() == "NAV_STATUS":
            msgstrl = str(msg).split(",")[1:2]
            valid_gps = int(msgstrl[0].split('=')[-1])

        r.sleep()
