# -*- coding: utf-8 -*-
"""
Created on Fri Mar 23 10:30:06 2018

@author: jchen3
"""
import clr
import System, math,time, gc,sys,subprocess
from System import Array
from System import SByte
import System.Reflection

try:
    sys.path.append("C:\\Program Files (x86)\\Analog Devices\\ADRV9010 Transceiver Evaluation Software FULL")
    System.Reflection.Assembly.LoadWithPartialName("adrv9010_dll.dll")
    import adrv9010_dll
    from adrv9010_dll import AdiEvaluationSystem
    from adrv9010_dll import Types
    from adrv9010_dll import Ad9528Types
except:
    print 'No Tokelau related environment found ...'
    
    
def sendTxData_Tokelau(Link,si,sq,tx_mask):
    tx_data = System.Collections.ArrayList()
    for i in xrange(4):
        tx_data.Add(System.Array[System.Int32](sq))
        tx_data.Add(System.Array[System.Int32](si))
    result = Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableGet(0,0)
    Link.Ads8.board.PerformTx(adrv9010_dll.FpgaTypes.adi_fpga9010_TxTollgateTrigSources_e.ADI_FPGA9010_TX_IMM_TRIG,tx_data,255)
    Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableSet(result[-2],tx_mask&0xF) 

    
def receiveRxOrxData_Tokelau(Link, rxChannel, time_ms):
    #Enables selected RX channel
    result = Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableGet(0,0)
    Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableSet(2**(rxChannel-1),result[-1])
    #Reads RX data from FPGA    
    capturedData = Link.Ads8.board.PerformRx(adrv9010_dll.FpgaTypes.adi_fpga9010_RxTollgateTrigSources_e.ADI_FPGA9010_IMM_TRIG,0xFF,time_ms,(1e3)) 
    return [list(capturedData[2*rxChannel-2]),list(capturedData[2*rxChannel-1])]
    


def do_ironscript(script_path):
    IRON_PYTHON_CMD = 'cd C:\\Program Files (x86)\\IronPython 2.7'    
    try:
        f = open(script_path,'r')
        f.close()
    except:
        print "No Init Script named " + script_path
        return -1
        
    try:
        print IRON_PYTHON_CMD+'&&ipy '+script_path
        res = subprocess.check_output(IRON_PYTHON_CMD+'&&ipy '+script_path , shell = True)
        print res
    except:
        print "Init Script running failure"
        return -2
        
    return 0



class Tokelau_MACRO(object):
    def __init__(self,Link):
        self.Link = Link
        self.Tokelau_LO1 = Types.adi_adrv9010_PllName_e.ADI_ADRV9010_LO1_PLL
        self.Tokelau_LO2 = Types.adi_adrv9010_PllName_e.ADI_ADRV9010_LO2_PLL
        self.Tokelau_Tx1 = Types.adi_adrv9010_TxChannels_e.ADI_ADRV9010_TX1
        self.Tokelau_Tx2 = Types.adi_adrv9010_TxChannels_e.ADI_ADRV9010_TX2
        self.Tokelau_Tx3 = Types.adi_adrv9010_TxChannels_e.ADI_ADRV9010_TX3
        self.Tokelau_Tx4 = Types.adi_adrv9010_TxChannels_e.ADI_ADRV9010_TX4
            





class TRx_Tokelau():
    def __init__(self, partnum='Tokelau'):
        self.ip = 0
        if partnum == 'adrv9010' or partnum == 'Tokelau':
            self.Link = AdiEvaluationSystem.Instance
            self.adrv9010 = self.Link.Adrv9010Get(1)
            self.partnum = 2    
            self.__name__ = 'Tokelau'
            self.Tokelau_TX_s = [0,Types.adi_adrv9010_TxChannels_e.ADI_ADRV9010_TX1,
                                    Types.adi_adrv9010_TxChannels_e.ADI_ADRV9010_TX2,
                                    Types.adi_adrv9010_TxChannels_e.ADI_ADRV9010_TX3,
                                    Types.adi_adrv9010_TxChannels_e.ADI_ADRV9010_TX4]

        
        self.MACRO = Tokelau_MACRO(self.Link)

            

    def transceiver_connect( self,ip='10.99.37.252' ):
        self.ip = ip
        try:
            self.Link.Ads8.board.Client.Connect(ip, 55556)
        except:
            print 'Connecting Tokelau Fail ...'
            return -1
        return 0
    
    
    def transceiver_disconnect(self):
        self.Link.Ads8.board.Client.Disconnect()
            
    
    def transceiverInit(self, conf_file=''):
        if conf_file == '':    
            print 'Empty Ironpython file !'
            return 1
        do_ironscript(conf_file)
        self.transceiver_connect(self.ip)
        return 0
    

    def spiRead(self,address):
        data = self.adrv9010.Hal.SpiByteRead(address, 0)
        print "SPI Read Address " + hex(address) + ": " + hex(data[1])
        return data[1]
        
         
    def spiWrite(self,address,data):
        self.adrv9010.Hal.SpiByteWrite(address, data)
        print "SPI Write Address " + hex(address) + ": " + hex(data)    

    
    def getAPI_ARMVersion(self):
        armvsn = Types.adi_adrv9010_ArmVersion_t()
        self.Link.Ads8.board.Adrv9010Device.Arm.ArmVersionGet(armvsn)
        print 'Tokelau Arm Version: %d.%d.%d(%s)' %(armvsn.majorVer,armvsn.minorVer,armvsn.rcVer,armvsn.armBuildType)
        apivsn = Types.adi_adrv9010_ApiVersion_t()
        self.Link.Ads8.board.Adrv9010Device.Control.ApiVersionGet(apivsn)
        print 'Tokelau API Version %d.%d.%d.%d' %(apivsn.siVer,apivsn.majorVer,apivsn.minorVer,apivsn.buildVer)
        return '{}.{}.{}.{}'.format(apivsn.siVer,apivsn.majorVer,apivsn.minorVer,apivsn.buildVer),\
                '{}.{}.{}.({})'.format(armvsn.majorVer,armvsn.minorVer,armvsn.rcVer,armvsn.armBuildType)
        
    
    
    def setLO_MHz(self,loname, freqMHz):
        TokelauLOSrc = [0,  Types.adi_adrv9010_PllName_e.ADI_ADRV9010_LO1_PLL,
                        Types.adi_adrv9010_PllName_e.ADI_ADRV9010_LO2_PLL,
                        Types.adi_adrv9010_PllName_e.ADI_ADRV9010_AUX_PLL]
        self.Link.Ads8.board.Adrv9010Device.RadioCtrl.PllFrequencySet(TokelauLOSrc[loname],int(freqMHz*1e6))


            
    def getLO_MHz(self,loname):
        TokelauLOSrc = [0,  Types.adi_adrv9010_PllName_e.ADI_ADRV9010_LO1_PLL,
                        Types.adi_adrv9010_PllName_e.ADI_ADRV9010_LO2_PLL,
                        Types.adi_adrv9010_PllName_e.ADI_ADRV9010_AUX_PLL]
        Freq_LOHz = self.Link.Ads8.board.Adrv9010Device.RadioCtrl.PllFrequencyGet(loname,0)[1]
        return Freq_LOHz/1e6
        

    def getTxRxORxSampleRate(self):
        initStruct = self.Link.Ads8.board.Adrv9010Device.InitStructGet()
        rx_kHz  = initStruct.rx.rxChannelCfg[0].profile.rxOutputRate_kHz
        orx_kHz = initStruct.rx.rxChannelCfg[4].profile.rxOutputRate_kHz
        tx_kHz  = initStruct.tx.txChannelCfg[0].profile.txInputRate_kHz
        return tx_kHz/1e3, rx_kHz/1e3, orx_kHz/1e3            

            
    def jesdbitwidth(self):
        initStruct = self.Link.Ads8.board.Adrv9010Device.InitStructGet()
        extendBit_Rx = initStruct.dataInterface.framer[0].jesd204Np
        extendBit_Tx = initStruct.dataInterface.deframer[0].jesd204Np
        return extendBit_Tx,extendBit_Rx


    def setRxTxEnable(self,rxorxmask,txmask):
        self.adrv9010.RadioCtrl.RxTxEnableSet(rxorxmask,txmask)
            
            
    def getRxTxEnable(self):    
        result = self.Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableGet(0,0)
        return result[-2],result[-1]
            
            
    def setTxAtt_dBm(self,channel,att_dBm):
        TxAtt = Types.adi_adrv9010_TxAtten_t()
        TxAtt.txChannelMask = 2**(channel-1)
        TxAtt.txAttenuation_mdB = att_dBm*1e3
        TxAttArr = Array[Types.adi_adrv9010_TxAtten_t]([TxAtt])
        self.adrv9010.Tx.TxAttenSet(TxAttArr,1) 
            

    def getTxAtt_dBm(self,channel):
        TxAtt = Types.adi_adrv9010_TxAtten_t()
        result = self.Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableGet(0,0)
        self.Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableSet(result[-2],0xF)
        self.adrv9010.Tx.TxAttenGet(self.Tokelau_TX_s[channel], TxAtt)        
        self.Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableSet(result[-2],result[-1])
        return TxAtt.txAttenuation_mdB /1.e3


    def NCOgen( self, freq_offset_KHz = 0, txchannel=1):
        NCO = Types.adi_adrv9010_TxTestToneCfg_t()
        NCO.txChannelMask = 2**(txchannel-1)
        NCO.enable = 1
        NCO.txToneFreq_Hz = int(freq_offset_KHz*1e3)
        NCO.txToneGain = Types.adi_adrv9010_TxNcoGain_e.ADI_ADRV9010_TX_NCO_NEG12_DB
        NCOArr = Array[Types.adi_adrv9010_TxTestToneCfg_t]([NCO])
        result = self.Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableGet(0,0)
        self.Link.Ads8.board.Adrv9010Device.RadioCtrl.RxTxEnableSet(result[-2],0xF)
        self.adrv9010.Tx.TxTestToneSet(NCOArr,1)


    def NCOstop(self):
        NCO = Types.adi_adrv9010_TxTestToneCfg_t()
        NCO.txChannelMask = 0x0F
        NCO.enable = 0    
        NCOArr = Array[Types.adi_adrv9010_TxTestToneCfg_t]([NCO])
        self.adrv9010.Tx.TxTestToneSet(NCOArr,1)            
    
    
    def setRxGain(self,channel,gainIndex):
        RxGain = Types.adi_adrv9010_RxGain_t()
        RxGain.rxChannelMask = 2**(channel-1)
        RxGain.gainIndex = gainIndex
        RxGainArr = Array[Types.adi_adrv9010_RxGain_t]([RxGain])
        self.adrv9010.Rx.RxGainSet(RxGainArr,1) 

            
    def getRxGain(self,channel):
        RxGain = Types.adi_adrv9010_RxGain_t()
        RxGain.rxChannelMask = 2**(channel-1)
        self.adrv9010.Rx.RxGainGet(2**(channel-1), RxGain)        
        return RxGain.gainIndex


    def receivedata(self,rx_channel,num):
        return receiveRxOrxData_Tokelau(self.Link,rx_channel,num/245760.)
    
    def receivedataTDD(self,rx_channel,num):
        pass
        
        
    def senddata(self,si,sq,txchannelMask):
        sendTxData_Tokelau(self.Link,si,sq,txchannelMask)
        gc.collect()
        

    def runInitCal(self,txChannelMsk,initCalMask):
        errorFlag = 0
        txCals = Types.adi_adrv9010_InitCals_t()
        txCals.calMask = int(initCalMask)
        txCals.channelMask = txChannelMsk
        txCals.warmBoot = 0
        self.Link.Ads8.board.Adrv9010Device.Cals.InitCalsRun(txCals)
        self.Link.Ads8.board.Adrv9010Device.Cals.InitCalsWait(1000, errorFlag)

        return errorFlag


    def gettemperature(self):
        tmp = self.Link.Ads8.board.Adrv9010Device.Gpio.TemperatureGet(0)[1]
        return tmp
            
            
    def clearerror(self):
        self.Link.Ads8.board.Adrv9010Device.Error.ErrorClear()
        
        
        
    def abortInitCal(self):
        self.Link.Ads8.board.Adrv9010Device.Cals.InitCalsAbort(0)
        

    def getdeframerstatus(self,deframer):
        if deframer == 1:   defSel =  Types.adi_adrv9010_DeframerSel_e.ADI_ADRV9010_DEFRAMER_1
        else: defSel =  Types.adi_adrv9010_DeframerSel_e.ADI_ADRV9010_DEFRAMER_0
        defStatus = Types.adi_adrv9010_DeframerStatus_t()
        self.Link.Ads8.board.Adrv9010Device.DataInterface.DeframerStatusGet(defSel,defStatus)
        print 'Tokelau Deframer %d status: 0x%x' %(deframer,defStatus.status) 
        return defStatus.status


    def getframerstatus(self,framer):
        if framer == 1:   fSel = Types.adi_adrv9010_FramerSel_e.ADI_ADRV9010_FRAMER_1
        elif framer == 2:   fSel = Types.adi_adrv9010_FramerSel_e.ADI_ADRV9010_FRAMER_2
        else:     fSel = Types.adi_adrv9010_FramerSel_e.ADI_ADRV9010_FRAMER_0
        Status = Types.adi_adrv9010_FramerStatus_t()
        self.Link.Ads8.board.Adrv9010Device.DataInterface.FramerStatusGet(fSel,Status)
        print 'Tokelau Framer %d status: 0x%x' %(framer,int(Status.status)) 
        return Status.status


    def getInitCalcomplete(self):
        isrun, err = 0,0
        self.Link.Ads8.board.Adrv9010Device.Cals.InitCalsCheckCompleteGet(isrun,err)
        return isrun,err
    
    
    
    def enabletrackingcals(self,msk):
        self.Link.Ads8.board.Adrv9010Device.Cals.TrackingCalsEnableSet(msk,Types.adi_adrv9010_TrackingCalEnableDisable_e.ADI_ADRV9010_TRACKING_CAL_ENABLE)

    

    def disabletrackingcals(self,msk):
        self.Link.Ads8.board.Adrv9010Device.Cals.TrackingCalsEnableSet(msk,Types.adi_adrv9010_TrackingCalEnableDisable_e.ADI_ADRV9010_TRACKING_CAL_DISABLE)



    def txpwrdetect_dBFs(self,channel):
        offset = 0x200*(channel-1)
        self.spiWrite(0x1e99+ offset, ((self.spiRead(0x1e99+ offset) & 0xFE)|1))
        self.spiWrite(0x1ea3+ offset,0)
        self.spiWrite(0x1ea4+ offset,0)
        val = self.spiRead(0x1ea4+ offset)<<8
        val += self.spiRead(0x1ea3+ offset)
       
        if val < 0.001:   return -1
        return 10*math.log10(val/32768.0)
    
    
    def getrssi(self,channel):
        pass
    
    def getpllstatus(self):
        return self.Link.Ads8.board.Adrv9010Device.RadioCtrl.PllStatusGet(0)[1]
    

    def getRxGainRange(self):
        rangeIndex = Types.adi_adrv9010_GainIndex_t() 
        self.Link.Ads8.board.Adrv9010Device.Rx.RxGainRangeGet(rangeIndex)
        return[rangeIndex.rx1MaxGainIndex,rangeIndex.rx1MinGainIndex,rangeIndex.rx2MaxGainIndex,rangeIndex.rx2MinGainIndex,
               rangeIndex.rx3MaxGainIndex,rangeIndex.rx3MinGainIndex,rangeIndex.rx4MaxGainIndex,rangeIndex.rx4MinGainIndex,
               rangeIndex.orx1orx2MaxGainIndex,rangeIndex.orx1orx2MinGainIndex,rangeIndex.orx3orx4MaxGainIndex,rangeIndex.orx3orx4MinGainIndex]
    

    def RadioOn(self):
        print("This function is only works in Talise. Do nothing in Tokelau")
        pass
    

    def RadioOff(self):
        print("This function is only works in Talise. Do nothing in Tokelau")
        pass


    def forcesend0enable(self,channel):
        pass
        
        
    def forcesend0disable(self,channel):
        pass
    
	
    def getPFIRcoeffs(self,channelMsk):	#channelMsk: bit0--Tx1  bit1--Tx2  bit2--Rx1  bit3--Rx2 
        pass
    
    def loopbackfromdeframerA2framer(self,framer,enable):
        pass
		
    def loopbackfromdeframerA2framer(self,framer,enable):
        pass
		
		
		
		