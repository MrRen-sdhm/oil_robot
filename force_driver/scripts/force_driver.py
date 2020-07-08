#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray

from socket import *
import struct

import numpy as np

import time

def InitSystem():
    targetHost = '192.168.0.108'
    targetPort = 4008
    BUFSIZ = 256

    if targetHost != "192.168.0.108":
    #print("\033[1;31;\033[0m")
        print("targetHost Error, please check!")
    if targetPort != 4008:
        print("targetPort Error, please check!")

    return targetHost, targetPort, BUFSIZ


class TCPClient(object):

    def __init__(self, targetHost, targetPort, BUFSIZ):

        self.targetHost = targetHost
        self.targetPort = targetPort
        self.BUFSIZ = BUFSIZ

        if self.targetHost != "192.168.0.108":
            #print("\033[1;31;\033[0m")
            print("targetHost Error, please check!")
        if self.targetPort != 4008:
            print("targetPort Error, please check!")

        self.ADDR = (self.targetHost, self.targetPort)
        self.tcpClientSock = socket(AF_INET, SOCK_STREAM)

    def TCPClient_Connect(self):
        print "[INFO] Connect to force sensor..."
        self.tcpClientSock.connect(self.ADDR)
        print "[INFO] Connected to force sensor."

    def TCPClient_Close(self):
        self.tcpClientSock.close()


class SRI(TCPClient):
    def __init__(self,targetHost, targetPort, BUFSIZ):
        super(SRI, self).__init__(targetHost, targetPort, BUFSIZ)

        self.Ex_uncompensated=[13.138298,13.138298,13.138298,13.138298,13.138298,13.138298]
        self.Ex_compensated=[5.006651,5.006651,5.006651,5.006651,5.006651,5.006651]
        self.AmpZero=[32775.000000,32757.000000,32758.000000,32701.000000,32756.000000,32802.000000]
        self.Gain=[123.691486,123.624210,123.801251,123.935803,123.780006,123.843741]
        #self.Compensation=[[2.870000],[1.500000],[-2.500000],[-0.200000],[-0.014000],[0.127000]]
        self.data_ft=[0, 0, 0, 0, 0, 0]

    def SRI_DecouplingCoefficient(self):
        Decoupled_r0 = [10.149000,4.464000,15.244000,448.840000,0.667000,-448.861000]
        Decoupled_r1 = [32.232000,-519.257000,-11.892000,263.949000,10.632000,265.259000]
        Decoupled_r2 = [1313.065000,-24.035000,1347.382000,23.366000,1328.592000,5.798000]
        Decoupled_r3 = [-2.971000,1.100000,-72.226000,0.366000,74.144000,-0.053000]
        Decoupled_r4 = [83.619000,-2.104000,-46.730000,-1.438000,-40.473000,-0.125000]
        Decoupled_r5 = [-0.281000,28.263000,2.587000,28.543000,-0.596000,27.906000]
        self.Decoupled=np.mat([Decoupled_r0,Decoupled_r1,Decoupled_r2,Decoupled_r3,Decoupled_r4,Decoupled_r5])


    def SRI_GetOneData(self,iscompensated = False):

        self.SRI_DecouplingCoefficient()
        #获取一次数据
        self.tcpClientSock.send(b'AT+GOD\r\n')
        #连续获取数据
        # self.tcpClientSock.send(b'AT+GSD\r\n')
        data_raw = self.tcpClientSock.recv(self.BUFSIZ)
        AD_Count = struct.unpack('>HHHHHH',data_raw[6:18])

        if iscompensated:
            self.data_ft[0] = 1000 * (AD_Count[0] - self.AmpZero[0]) / 65535 * 5 / self.Gain[0] / self.Ex_compensated[0]
            self.data_ft[1] = 1000 * (AD_Count[1] - self.AmpZero[1]) / 65535 * 5 / self.Gain[1] / self.Ex_compensated[1]
            self.data_ft[2] = 1000 * (AD_Count[2] - self.AmpZero[2]) / 65535 * 5 / self.Gain[2] / self.Ex_compensated[2]
            self.data_ft[3] = 1000 * (AD_Count[3] - self.AmpZero[3]) / 65535 * 5 / self.Gain[3] / self.Ex_compensated[3]
            self.data_ft[4] = 1000 * (AD_Count[4] - self.AmpZero[4]) / 65535 * 5 / self.Gain[4] / self.Ex_compensated[4]
            self.data_ft[5] = 1000 * (AD_Count[5] - self.AmpZero[5]) / 65535 * 5 / self.Gain[5] / self.Ex_compensated[5]

            DATA_FT = np.mat([[self.data_ft[0]],[self.data_ft[1]],[self.data_ft[2]],[self.data_ft[3]],[self.data_ft[4]],[self.data_ft[5]]])

            FT = self.Decoupled*DATA_FT
            #FT = FT+self.Compensation
        else:
            self.data_ft[0] = 1000 * (AD_Count[0] - self.AmpZero[0]) / 65535 * 5 / self.Gain[0] / self.Ex_uncompensated[0]
            self.data_ft[1] = 1000 * (AD_Count[1] - self.AmpZero[1]) / 65535 * 5 / self.Gain[1] / self.Ex_uncompensated[1]
            self.data_ft[2] = 1000 * (AD_Count[2] - self.AmpZero[2]) / 65535 * 5 / self.Gain[2] / self.Ex_uncompensated[2]
            self.data_ft[3] = 1000 * (AD_Count[3] - self.AmpZero[3]) / 65535 * 5 / self.Gain[3] / self.Ex_uncompensated[3]
            self.data_ft[4] = 1000 * (AD_Count[4] - self.AmpZero[4]) / 65535 * 5 / self.Gain[4] / self.Ex_uncompensated[4]
            self.data_ft[5] = 1000 * (AD_Count[5] - self.AmpZero[5]) / 65535 * 5 / self.Gain[5] / self.Ex_uncompensated[5]

            DATA_FT = np.mat([[self.data_ft[0]],[self.data_ft[1]],[self.data_ft[2]],[self.data_ft[3]],[self.data_ft[4]],[self.data_ft[5]]])

            FT = self.Decoupled*DATA_FT

        #print("Sensor One Data:")
        #print("=================")
        #print(FT)
        #print("=================\n")

        return FT

    def SRI_Config(self):
        Ex, Gain, AmpZero=[], [], []

        #AT+EXMV, get Ex
        self.tcpClientSock.send(b'AT+EXMV=?\r\n')
        server_return = self.tcpClientSock.recv(self.BUFSIZ)

        print("\n[Info]:Client->AT+EXMV,Server return:",server_return)
        Ex_strData=server_return.decode("utf-8")[len(b'AT+EXMV=')+1:-5]

        for i in range(len(Ex_strData.split(';'))):
            Ex_data = float(Ex_strData.split(';')[i])
            Ex.append(Ex_data)

        #AT+CHNAPG, get Gain
        self.tcpClientSock.send(b'AT+CHNAPG=?\r\n')
        server_return = self.tcpClientSock.recv(self.BUFSIZ)

        print("\n[Info]:Client->AT+CHNAPG,Server return:",server_return)
        Gain_strData=server_return.decode("utf-8")[len(b'AT+CHNAPG=')+1:-5]

        for i in range(len(Gain_strData.split(';'))):
            Gain_data = float(Gain_strData.split(';')[i])
            Gain.append(Gain_data)

        #AT+AMPZ, get AmpZero
        self.tcpClientSock.send(b'AT+AMPZ=?\r\n')
        server_return = self.tcpClientSock.recv(self.BUFSIZ)

        print("\n[Info]:Client->AT+AMPZ,Server return:",server_return)
        AmpZero_strData=server_return.decode("utf-8")[len(b'AT+AMPZ=')+1:-5]

        for i in range(len(Gain_strData.split(';'))):
            AmpZero_data = float(AmpZero_strData.split(';')[i])
            AmpZero.append(AmpZero_data)

        #AT+DCPCU get unit
        self.tcpClientSock.send(b'AT+DCPCU=MVPV\r\n')
        server_return = self.tcpClientSock.recv(self.BUFSIZ)
        print("\n[Info]:Client->AT+DCPCU,Server return:",server_return)

        #AT+SGDM get (A01,A02,A03,A04,A05,A06);C;1;(WMA:1.000000)
        self.tcpClientSock.send(b'AT+SGDM=?\r\n')
        server_return = self.tcpClientSock.recv(self.BUFSIZ)
        print("\n[Info]:Client->AT+SGDM,Server return:",server_return)

        #AT+SMPF get SampleFreq
        self.tcpClientSock.send(b'AT+SMPF=?\r\n')
        server_return = self.tcpClientSock.recv(self.BUFSIZ)
        print("\n[Info]:Client->AT+SMPF,Server return:",server_return)

        print('Config System Finshed...')
        return Ex, Gain, AmpZero



if __name__ == "__main__":
    rospy.init_node('force_server')

    targetHost, targetPort, BUFSIZ = InitSystem()
    operate_sensor = SRI(targetHost, targetPort, BUFSIZ)

    operate_sensor.TCPClient_Connect()


    pub = rospy.Publisher('force', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    data_out = Float64MultiArray()
    while not rospy.is_shutdown():
        data = operate_sensor.SRI_GetOneData(iscompensated = True)

        data = np.array(data)  # (6, 1)
        data = data.reshape(-1) # (1, 6)

        data_out.data = data

        pub.publish(data_out)
        
        # print(data_out.data)  # x, y, z, mx, my, mz / N / N*M

        rate.sleep()

    operate_sensor.TCPClient_Close()



'''
    #视觉引导结束后
    #机械臂向左/向右移动一定距离
    force_z_Value = 555
    if abs(data[2,0]) > force_z_Value:
        print("对接成功")
    else：
        #根据实际情况移动机械臂
        #视觉重新定位
'''

