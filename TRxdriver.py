# -*- coding: utf-8 -*-
"""
Created on Fri Mar 23 10:49:40 2018

@author: jchen3
"""

from __future__ import division, print_function   
    

class TRx_family():
    def __init__(self, partnumber = ''):
        self.trx = 0
        print(u'You are using framework Scorpio(天蝎座)')
        if partnumber == 'adrv9010' or partnumber == 'Tokelau':   
            try:
                import driver_tokelau
            except:
                print ('Loading driver: Tokelau driver not found ...'  )     
            self.trx = driver_tokelau.TRx_Tokelau()
            
        elif partnumber == 'ad9379' or partnumber == 'Talise': 
            try:
                import driver_talise
            except:
                print ('Loading driver: Talise driver not found ...')
            self.trx = driver_talise.TRx_Talise()
            
        elif partnumber == 'ad9370' or partnumber == 'Mykonos':
            pass
        
        
        
        
        
        
        


