# -*- coding: utf-8 -*-
"""
Created on Fri Mar 23 11:08:09 2018

@author: jchen3
"""
from __future__ import division, print_function
import clr
import System, math,time, gc,sys,subprocess
from System import Array
from System import SByte
import System.Reflection
import numpy as np



try:
    sys.path.append("C:\\Program Files (x86)\\Analog Devices\\ADRV9009 Transceiver Evaluation Software")
    clr.AddReference('AdiCmdServerClient')
    import AdiCmdServerClient
    from AdiCmdServerClient import AdiCommandServerClient
    from AdiCmdServerClient import Talise
    from AdiCmdServerClient import FpgaTalise 
except:
    print('No Talise related environment found ...')

def sendTxData_Talise(Link,si,sq,TX1=1,TX2=1):    
    tx1data = []
    tx2data = []
#    extendBit1 = (Link.SpiRead(System.UInt16(0x1588)) &0x1F) + 1
#    extendBit2 = (Link.SpiRead(System.UInt16(0x15d8)) &0x1F) + 1
#    extendBit  = max(extendBit1,extendBit2)
    Link.FpgaTalise.EnableTxDataPaths(Link.FpgaTalise.TxDataPath.Disable)
    if TX1 and TX2:
        for i in range(len(si)):    tx1data.extend([si[i],sq[i]])
        tx1data = Array[System.Int16](tx1data)
        Link.WriteRam(Link.FpgaTalise.FpgaChannel.Tx1, 0, tx1data)   #No description of this funciton in chm, nor the first parameter type
        Link.FpgaTalise.SetTxTransmitSamples(Link.FpgaTalise.TxBuffer.Tx1DataMover, len(tx1data))
        for i in range(len(si)):    tx2data.extend([si[i],sq[i]])
        tx2data = Array[System.Int16](tx2data)
        Link.WriteRam(Link.FpgaTalise.FpgaChannel.Tx2, 0, tx2data)
        Link.FpgaTalise.SetTxTransmitSamples(Link.FpgaTalise.TxBuffer.Tx2DataMover, len(tx2data))

        Link.FpgaTalise.EnableTxDataPaths(Link.FpgaTalise.TxDataPath.Tx1Tx2)
        Link.Talise.SetRxTxEnable(Talise.RxOrxChannel.RxOff, Talise.TxChannel.Tx1Tx2)

    
    elif TX1:
        for i in range(len(si)):    tx1data.extend([si[i],sq[i]])
        tx1data = Array[System.Int16](tx1data)
        Link.WriteRam(Link.FpgaTalise.FpgaChannel.Tx1, 0, tx1data)   #No description of this funciton in chm, nor the first parameter type
        Link.FpgaTalise.SetTxTransmitSamples(Link.FpgaTalise.TxBuffer.Tx1DataMover, len(tx1data))
        Link.FpgaTalise.EnableTxDataPaths(Link.FpgaTalise.TxDataPath.Tx1)
        Link.Talise.SetRxTxEnable(Talise.RxOrxChannel.RxOff, Talise.TxChannel.Tx1)


    elif TX2:
        for i in range(len(si)):    tx2data.extend([si[i],sq[i]])
        tx2data = Array[System.Int16](tx2data)
        Link.WriteRam(Link.FpgaTalise.FpgaChannel.Tx2, 0, tx2data)
        Link.FpgaTalise.SetTxTransmitSamples(Link.FpgaTalise.TxBuffer.Tx2DataMover, len(tx2data))
        Link.FpgaTalise.EnableTxDataPaths(Link.FpgaTalise.TxDataPath.Tx2)
        Link.Talise.SetRxTxEnable(Talise.RxOrxChannel.RxOff, Talise.TxChannel.Tx2)

    
    else:
        print ("Both transmitters have been disabled!")

    Link.FpgaTalise.SetTxTrigger(Link.FpgaTalise.TxTrigger.Immediate)
    Link.FpgaTalise.SetTxTransmitMode(1)
    Link.FpgaTalise.StartTxData()
    print ("TX DATA SENT!!") 
    Link.Talise.RadioOn()


def receiveRxOrxData_Talise(Link,rx_channel = 1,num = 245760):
    if rx_channel == 1:
        fpgaChannel = Link.FpgaTalise.FpgaChannel.Rx1
        dataMover = Link.FpgaTalise.RxCapture.Rx1DataMover
        rxDataPath = Link.FpgaTalise.RxDataPath.Rx1
    elif rx_channel == 2:
        fpgaChannel = Link.FpgaTalise.FpgaChannel.Rx2
        dataMover = Link.FpgaTalise.RxCapture.Rx2DataMover
        rxDataPath = Link.FpgaTalise.RxDataPath.Rx2 
    elif rx_channel == 0x10:
        fpgaChannel = Link.FpgaTalise.FpgaChannel.ObsRx1
        dataMover = Link.FpgaTalise.RxCapture.ObsRx1DataMover
        rxDataPath = Link.FpgaTalise.RxDataPath.ObsRx1   
    elif rx_channel == 0x20:
        fpgaChannel = Link.FpgaTalise.FpgaChannel.ObsRx2
        dataMover = Link.FpgaTalise.RxCapture.ObsRx2DataMover
        rxDataPath = Link.FpgaTalise.RxDataPath.ObsRx2   
    Link.FpgaTalise.EnableRxDataPaths(Link.FpgaTalise.RxDataPath.Disable)
    Link.FpgaTalise.SetRxCaptureSamples(dataMover, System.UInt32(num))
    Link.FpgaTalise.SetRxTrigger(Link.FpgaTalise.RxTrigger.Immediate)
    Link.FpgaTalise.EnableRxDataPaths(rxDataPath)
    Link.FpgaTalise.CaptureRxData()
    Link.FpgaTalise.WaitRxCapture(5000)
    res = list( Link.ReadRam(fpgaChannel, System.UInt32(0), System.Int32(num)) )
    return [res[::2],res[1::2]]


def receiveRxOrxDataTDD_Talise(Link,rx_channel = 1,num = 245760):
    if rx_channel == 1:
        fpgaChannel = Link.FpgaTalise.FpgaChannel.Rx1
        dataMover = Link.FpgaTalise.RxCapture.Rx1DataMover
        rxDataPath = Link.FpgaTalise.RxDataPath.Rx1
    elif rx_channel == 2:
        fpgaChannel = Link.FpgaTalise.FpgaChannel.Rx2
        dataMover = Link.FpgaTalise.RxCapture.Rx2DataMover
        rxDataPath = Link.FpgaTalise.RxDataPath.Rx2 
    elif rx_channel == 0x10:
        fpgaChannel = Link.FpgaTalise.FpgaChannel.ObsRx1
        dataMover = Link.FpgaTalise.RxCapture.ObsRx1DataMover
        rxDataPath = Link.FpgaTalise.RxDataPath.ObsRx1   
    elif rx_channel == 0x20:
        fpgaChannel = Link.FpgaTalise.FpgaChannel.ObsRx2
        dataMover = Link.FpgaTalise.RxCapture.ObsRx2DataMover
        rxDataPath = Link.FpgaTalise.RxDataPath.ObsRx2   
    Link.FpgaTalise.EnableRxDataPaths(Link.FpgaTalise.RxDataPath.Disable)
    Link.FpgaTalise.SetRxCaptureSamples(dataMover, System.UInt32(num))
    Link.FpgaTalise.SetRxTrigger(Link.FpgaTalise.RxTrigger.TddStateMachinePulse)
    Link.FpgaTalise.EnableRxDataPaths(rxDataPath)
    Link.FpgaTalise.CaptureRxData()
    Link.FpgaTalise.WaitRxCapture(5000)
    res = list( Link.ReadRam(fpgaChannel, System.UInt32(0), System.Int32(num)) )
    return [res[::2],res[1::2]]


def do_ironscript(script_path):
    IRON_PYTHON_CMD = 'cd C:\\Program Files (x86)\\IronPython 2.7'    
    try:
        f = open(script_path,'r')
        f.close()
    except:
        print ("No Init Script named " + script_path)
        return -1
        
    try:
        print (IRON_PYTHON_CMD+'&&ipy '+script_path)
        res = subprocess.check_output(IRON_PYTHON_CMD+'&&ipy '+script_path , shell = True)
        print (res)
    except:
        print ("Init Script running failure")
        return -2
        
    return 0


    
class Talise_MACRO(object):
    def __init__(self,Link):
        self.Link = Link
        self.Talise_RFPLL  = self.Link.Talise.PllName.RfPll
        self.Talise_AUXPLL = self.Link.Talise.PllName.AuxPll
        self.Talise_TX1    = self.Link.Talise.TxChannel.Tx1
        self.Talise_TX2      = self.Link.Talise.TxChannel.Tx2
        self.Talise_TX1TX2     = self.Link.Talise.TxChannel.Tx1Tx2
        
        self.Rx1QEC_tracking       = self.Link.Talise.TrackingCalMask.Rx1Qec
        self.Rx2QEC_tracking       = self.Link.Talise.TrackingCalMask.Rx2Qec
        self.Tx1LOL_tracking       = self.Link.Talise.TrackingCalMask.Tx1Lol
        self.Tx2LOL_tracking       = self.Link.Talise.TrackingCalMask.Tx2Lol



class TRx_Talise(object):
    def __init__(self, partnum='Talise'):
        self.partnum = partnum     #self.partnum  0--Mykonos   1-- Talise    2-- Tokelau
        self.ip = 0
        if partnum == 'ad9379' or partnum == 'Talise':
            self.Link = AdiCommandServerClient.Instance
            self.partnum = 1
            self.__name__ = 'Talise'
        
        self.MACRO = Talise_MACRO(self.Link)

            

    def transceiver_connect( self,ip='10.99.37.251' ):
        self.ip = ip
        if self.Link.hw.Connected == 1:    pass
        else:
            if ip == '':    
                print ('Empty IP Address, Please check')
                return -1
            try:
                self.Link.hw.Connect(ip, 55555)
            except:
                print ('Connecting Talise Fail ...')
                return -2

        return 0
    
    
    def transceiver_disconnect(self):
        self.Link.hw.Disconnect()

            
    
    def transceiverInit(self, conf_file=''):
        if conf_file == '':    
            print ('Empty Ironpython file !')
            return 1
        do_ironscript(conf_file)
        self.transceiver_connect(self.ip)
        return 0
    

    def spiRead(self,address):
        data = self.Link.SpiRead(System.UInt16(address))&0xFF
        print ("SPI Read Address " + hex(address) + ": " + hex(data))
        return data
        
         
    def spiWrite(self,address,data):
        self.Link.SpiWrite( System.UInt16(address), System.Byte(data) )
        print ("SPI Write Address " + hex(address) + ": " + hex(data))   

    
    def getAPI_ARMVersion(self):
        apivsn = self.Link.Talise.GetApiVersion()
        armvsn = self.Link.Talise.GetArmVersion()
        print( 'Talise Arm Version: %s' %armvsn)
        print( 'Talise API Version: %s' %apivsn)
        return apivsn,armvsn
    
    
    def setLO_MHz(self,loname, freqMHz):           
        TaliseLOSrc = [0,self.Link.Talise.PllName.RfPll,self.Link.Talise.PllName.AuxPll]
        state = self.Link.Talise.GetRadioState(0)
        if state == 3:    self.Link.Talise.RadioOff()
        self.Link.Talise.SetRfPllFrequency(TaliseLOSrc[loname], int(freqMHz*1e6))
        if state == 3:   self.Link.Talise.RadioOn()

            
    def getLO_MHz(self,loname):
        Freq_LOHz = self.Link.Talise.GetRfPllFrequency(loname, 0)
        return Freq_LOHz/1e6
        

    def getTxRxORxSampleRate(self):
        addr_TxConf = System.UInt16(0x101)
        addr_RxConf = System.UInt16(0x102)
        addr_OxConf = System.UInt16(0x103)
        addr_Rxfir  = System.UInt16(0x542)
        addr_Oxfir  = System.UInt16(0x543)
        dic_clk = {16:1966.08, 8:1228.8, 20:1843.2}
        clkint = ((self.Link.SpiRead(System.UInt16(0x211)) & 0x7)<<8) + self.Link.SpiRead(System.UInt16(0x210))
        reg_val_tx = self.Link.SpiRead(addr_TxConf)&0x1F
        reg_val_rx = self.Link.SpiRead(addr_RxConf)&0xFF
        reg_val_ox = self.Link.SpiRead(addr_OxConf)&0xFF
        reg_rxfir  = self.Link.SpiRead(addr_Rxfir) &0x3
        reg_oxfir  = self.Link.SpiRead(addr_Oxfir) &0x3
        if clkint !=20:
            txgain = dic_clk[clkint]
            if reg_val_tx & 0x4:         txgain/=2.0
            if reg_val_tx & 0x8:         txgain/=2.0
            if reg_val_tx & 0x10:        txgain/=2.0
        else:   txgain = 368.64
        if reg_val_tx & 0x03 == 2:   txgain/=2.0
        elif reg_val_tx & 0x03 == 3: txgain/=4.0
      
        rxgain = dic_clk[clkint]
        if   reg_rxfir == 2:      rxgain/=2.0
        elif reg_rxfir == 3:      rxgain/=4.0
        if reg_val_rx & 0x1F:     rxgain/=2.0
        if reg_val_rx & 0x2:      rxgain/=5.0
        else:                     rxgain/=4.0   

        oxgain = dic_clk[clkint]
        if   reg_oxfir == 2:      oxgain/=2.0
        elif reg_oxfir == 3:      oxgain/=4.0   
        if reg_val_rx & 0x2:      oxgain/=5.0
        else:                     oxgain/=4.0 
        if reg_val_ox & 0x4:      oxgain/=2.0
        return txgain,rxgain,oxgain
        
         

            
    def jesdbitwidth(self):
        extendBit1 = (self.Link.SpiRead(System.UInt16(0x1588)) &0x1F) + 1
        extendBit2 = (self.Link.SpiRead(System.UInt16(0x15d8)) &0x1F) + 1
        extendBit_Tx  = max(extendBit1,extendBit2)
        
        extendBit1 = (self.Link.SpiRead(System.UInt16(0x1528)) &0x1F) + 1
        extendBit2 = (self.Link.SpiRead(System.UInt16(0x1550)) &0x1F) + 1
        extendBit_Rx  = max(extendBit1,extendBit2) 

        return extendBit_Tx,extendBit_Rx


    def setRxTxEnable(self,rxorxmask,txmask):
        rx_s = [self.Link.Talise.RxOrxChannel.RxOff,self.Link.Talise.RxOrxChannel.Rx1,
                self.Link.Talise.RxOrxChannel.Rx2,self.Link.Talise.RxOrxChannel.Rx1Rx2]
        orx_s = [self.Link.Talise.RxOrxChannel.Orx1,self.Link.Talise.RxOrxChannel.Orx2,self.Link.Talise.RxOrxChannel.Orx1Orx2]
        tx_s = [self.Link.Talise.TxChannel.TxOff,self.Link.Talise.TxChannel.Tx1,self.Link.Talise.TxChannel.Tx2,self.Link.Talise.TxChannel.Tx1Tx2]
        self.Link.Talise.SetRxTxEnable(rx_s[rxorxmask&3], tx_s[txmask])
        orxmask = rxorxmask>>2
        if orxmask !=0:  self.Link.Talise.SetRxTxEnable(orx_s[orxmask&3], tx_s[txmask])

            
            
    def getRxTxEnable(self):    
        result = self.Link.Talise.GetRxTxEnable(0,0)
        return result[-2],result[-1]
            
            
    def setTxAtt_dBm(self,channel,att_dBm):
        if channel == 1 or channel== 3:
            self.Link.Talise.SetTxAttenuation(self.Link.Talise.TxChannel.Tx1,System.Double(att_dBm))
        if channel == 2 or channel== 3:
            self.Link.Talise.SetTxAttenuation(self.Link.Talise.TxChannel.Tx2,System.Double(att_dBm))

            

    def getTxAtt_dBm(self,channel):
        att_dBm = 0
        result = self.Link.Talise.GetRxTxEnable(0,0)#ironpython just has 2 return val
        rxstate = result[-2]
        txstate = result[-1]
        a = System.Double(0)
        self.Link.Talise.SetRxTxEnable(rxstate, self.Link.Talise.TxChannel.Tx1Tx2)
        if channel == 1:
            att_dBm = self.Link.Talise.GetTxAttenuation(self.Link.Talise.TxChannel.Tx1,a)
        elif channel == 2:
            att_dBm = self.Link.Talise.GetTxAttenuation(self.Link.Talise.TxChannel.Tx2,a)
        self.Link.Talise.SetRxTxEnable(rxstate, txstate)
        return att_dBm/1.e3



    def NCOgen( self, freq_offset_KHz = 0, txchannel=1):
        self.Link.Talise.RadioOff()
        if txchannel == 3:
            self.Link.Talise.SetRxTxEnable(self.Link.Talise.RxOrxChannel.Orx1,self.Link.Talise.TxChannel.Tx1Tx2)
        elif txchannel == 1:
            self.Link.Talise.SetRxTxEnable(self.Link.Talise.RxOrxChannel.Orx1,self.Link.Talise.TxChannel.Tx1)
        elif txchannel == 2:
            self.Link.Talise.SetRxTxEnable(self.Link.Talise.RxOrxChannel.Orx1,self.Link.Talise.TxChannel.Tx2)
        NCO = AdiCmdServerClient.TxNcoTestToneCfg()
        NCO.Enable = 1
        NCO.Tx1ToneFreq_kHz = freq_offset_KHz
        NCO.Tx2ToneFreq_kHz = freq_offset_KHz
        self.Link.Talise.EnableTxNco(NCO)
        #    Link.SpiWrite(0xE8E,5)
        self.Link.SpiWriteField(0xe8e, 5,0xF,0)
        self.Link.Talise.SetTxAttenuation(self.Link.Talise.TxChannel.Tx1,12)
        self.Link.Talise.SetTxAttenuation(self.Link.Talise.TxChannel.Tx2,12)
        self.Link.Talise.RadioOn()



    def NCOstop(self):
        NCO = AdiCmdServerClient.TxNcoTestToneCfg()
        NCO.Enable = 0
        self.Link.Talise.EnableTxNco(NCO)
           
    
    
    def setRxGain(self,channel,gainIndex):
        if channel & 0x01:  self.Link.Talise.SetRxManualGain(self.Link.Talise.RxChannel.Rx1,gainIndex)
        if channel & 0x02:  self.Link.Talise.SetRxManualGain(self.Link.Talise.RxChannel.Rx2,gainIndex)
        if channel & 0x10:  self.Link.Talise.SetRxManualGain(self.Link.Talise.RxChannel.ORx1,gainIndex)
        if channel & 0x20:  self.Link.Talise.SetRxManualGain(self.Link.Talise.RxChannel.ORx2,gainIndex)


            
    def getRxGain(self,channel):
        gainIndex = 0
        if channel & 0x01:  self.Link.Talise.GetRxManualGain(self.Link.Talise.RxChannel.Rx1,gainIndex)
        if channel & 0x02:  self.Link.Talise.GetRxManualGain(self.Link.Talise.RxChannel.Rx2,gainIndex)
        if channel & 0x10:  self.Link.Talise.GetRxManualGain(self.Link.Talise.RxChannel.ORx1,gainIndex)
        if channel & 0x20:  self.Link.Talise.GetRxManualGain(self.Link.Talise.RxChannel.ORx2,gainIndex)
        return gainIndex

    def rxdatanormalize(self,si,sq):
        return np.array(si)/32768, np.array(sq)/32768

    def receivedata(self,rx_channel,num):
        return receiveRxOrxData_Talise(self.Link,rx_channel,num)
    
    
    def receivedataTDD(self,rx_channel,num):
        return receiveRxOrxDataTDD_Talise(self.Link,rx_channel,num)
    
    def txdatascaler(self,si,sq,bitnum=14):
        # 32767 / 4 = 8191.75 = 8192 rounded * 4 = 32768 > max codes
        MAXSCALINGFACTOR = 32764
        si = (si* MAXSCALINGFACTOR)//4 * 4 
        sq = (sq* MAXSCALINGFACTOR)//4 * 4 
        if bitnum==14:    return si, sq
        else:             return si//16, sq//16

    def senddata(self,si,sq,txchannelMask):
        sendTxData_Talise(self.Link,si,sq, txchannelMask & 1, (txchannelMask>>1)&1 )
        gc.collect()
        

    def runInitCal(self,txChannelMsk,initCalMask):
        errorFlag = 0
        self.Link.Talise.RadioOff()
        self.Link.Talise.AbortInitCals(0)
        self.Link.Talise.RunInitCals(initCalMask)
        self.Link.Talise.WaitInitCals(20000, 0)
        return errorFlag


    def gettemperature(self):
        tmp = 0
        tmp = self.Link.Talise.GetTemperature(0)
        return tmp
            
            
    def clearerror(self):
        pass
        
        
    def abortInitCal(self):
        self.Link.Talise.AbortInitCals(0)
        

    def getdeframerstatus(self,deframer):   
        if deframer == 1:   defSel = self.Link.Talise.DeframerSelect.DeframerB
        else:               defSel = self.Link.Talise.DeframerSelect.DeframerA
        status = self.Link.Talise.ReadDeframerStatus(defSel)
        print ('Talise Deframer %d status: 0x%x' (deframer,status) )
        return status



    def getframerstatus(self,framer):
        if framer == 1: fSel = self.Link.Talise.DeframerSelect.FramerB 
        else:           fSel = self.Link.Talise.DeframerSelect.FramerA 
        status = self.Link.Talise.ReadFramerStatus(fSel)
        print ('Talise Framer %d status: 0x%x' (framer,status) )
        return status



    def getInitCalcomplete(self):
        isrun, err = 0,0
        isrun,err = self.Link.Talise.CheckInitCalComplete(isrun,err)

    
    
    
    def enabletrackingcals(self,msk):
        self.Link.Talise.EnableTrackingCals(msk)

    

    def disabletrackingcals(self,msk):
        self.Link.Talise.EnableTrackingCals(msk)



    def txpwrdetect_dBFs(self,channel):
        val = self.Link.SpiRead(0x1660)
        if channel == 1:    self.Link.SpiWrite(0x1660,(val&0xDF)|1)
        else:               self.Link.SpiWrite(0x1660,val|0x21) 
        self.Link.SpiWrite(0x1669,0)
        self.Link.SpiWrite(0x166a,0)
        val = self.Link.SpiRead(0x166a)<<8
        val+= self.Link.SpiRead(0x1669)
        
        if val < 0.001:   return -1
        return 10*math.log10(val/32768.0)
        


    def getrssi(self,channel):
        if channel > 2 or channel <1:
            print ('Invalid Rx channel for Talise')
            return -1
        self.SpiWrite(0x745+channel,0)
        return self.SpiRead(0x745+channel)


    def getpllstatus(self):
        return self.Link.Talise.GetPllsLockStatus()


    def getRxGainRange(self):
        maxgainindex_rx1 = self.spiRead(0x5d7)&0xFF
        mingainindex_rx1 = self.spiRead(0x5d8)&0xFF
        maxgainindex_rx2 = self.spiRead(0x5e7)&0xFF
        mingainindex_rx2 = self.spiRead(0x5e8)&0xFF
        maxgainindex_ox1 = self.spiRead(0x5d7)&0xFF
        mingainindex_ox1 = self.spiRead(0x5d8)&0xFF

        return [maxgainindex_rx1,mingainindex_rx1,maxgainindex_rx2,
                mingainindex_rx2,maxgainindex_ox1,mingainindex_ox1]
        

    def RadioOn(self):
        self.Link.Talise.RadioOn()

        
    def RadioOff(self):
        self.Link.Talise.RadioOff()
        
        
    def forcesend0enable(self,channel):
        if (channel&1) == 1:    self.spiWrite(0x1628,3)
        if channel & 2:         self.spiWrite(0x1629,3)
        
        
    def forcesend0disable(self,channel):
        if (channel&1) == 1:    self.spiWrite(0x1628,0)
        if channel & 2:         self.spiWrite(0x1629,0)        
        
        
    def rescheduletracking(self,calmask):
        self.Link.Talise.RescheduleTrackingCal(calmask)
        
    def getPFIRcoeffs(self,channelMsk):    #channelMsk: bit0--Tx1  bit1--Tx2  bit2--Rx1  bit3--Rx2 
        #read FIR Coefficients
        if channelMsk <= 2:    numcoff = 20
        elif channelMsk<=8:    numcoff = 48
        else:                  numcoff = 24
        self.SpiWrite(0xC0, 0x80 | channelMsk)
        coff_s = []
        for i in range(numcoff):
            self.SpiWrite(0xC3,i)
            coff_s.append(self.Link.SpiRead(0xC2))
        return coff_s    
        
    def loopbackfromdeframerA2framer(self,framer,enable):
        if framer == 1: address = 0x155A #Framer B
        else:           address = 0x1532 #Framer A
        data = self.Link.SpiRead(System.UInt16(address))&0xFF
        if enable:    self.Link.SpiWrite(System.UInt16(address),System.Byte(data | 0x80))
        else:         self.Link.SpiWrite(System.UInt16(address),System.Byte(data & 0x7F))
        
        
    def loopbackfromrx2tx(self,channelMsk,enable): 
        data = self.Link.SpiRead(System.UInt16(0x101))&0xFF
        if   channelMsk&1 and enable:
            self.Link.SpiWrite(System.UInt16(0x101),System.Byte(data | 0x40))
        elif channelMsk&2 and enable:	
            self.Link.SpiWrite(System.UInt16(0x101),System.Byte(data | 0x80))
        elif channelMsk&1 and enable == 0:	
            self.Link.SpiWrite(System.UInt16(0x101),System.Byte(data & 0xBF))
        elif channelMsk&2 and enable == 0:	
            self.Link.SpiWrite(System.UInt16(0x101),System.Byte(data & 0x7F))
        else:
            print ('Invalid Tx channel for Talise')










			