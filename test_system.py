import spade
from os.path import dirname, join
from os import mkdir
import os
from pypower.loadcase import loadcase
from math import exp
import numpy as np

import pandas as pd 
import time
import math
import matplotlib.pyplot as plt
import datetime
import dateutil, pylab,random  
from pylab import *  
from pypower.rundcpf import rundcpf
import matplotlib as mpl
from scipy.optimize import minimize
from pypower.savecase import savecase
import traceback
import logging
'''
agent behavior: including communication and control algorithm
'''
 
class MyAgent(spade.Agent.Agent):
    def __init__(self, agentid, n):         #, B
        self.d = None
        self.agentid = agentid
        self.test_consen = [0]
        self.test = [0]
        today = datetime.datetime.now()
        self.xaxe = time.mktime(today.timetuple())
        self.time = 0  
        self.powerout = 0 # MG's powerexchange with outside, from microcontrol 
        #self.pf = B.pf  # power flow on the distribution of the system, a list, from outside class MyPF 
        number = 13
        self.load = [0]
        self.parray = [0]
        self.comarray = [0] 
        self.pmax = [0]
        self.pmin = [0] 
        self.psum = [0]
        self.n = n    #n starts from 1
        self.request = [0]
        self.wind2_out = [0]
        self.fc_out = [0]
        self.pv2_out = [0] 
        self.com12 = [0]
        self.com15 = [0]   
        self.soc = [0] 
        self.mark_iso = [0]
        self.send_trag = 0
        self.feedback_mark = 0               
        spade.Agent.Agent.__init__(self, self.agentid, "secret")

        # interupt detection
        self.e1 = [0]
        self.e2 = [0]    
        self.e3 = [0]
        self.e4 = [0] 
        self.e5 = [0]
        self.e6 = [0]    
        self.e7 = [0]
        self.e8 = [0] 
        self.e9 = [0]
        self.e10 = [0]    
        self.e11 = [0]
        self.e12 = [0]         
    class Receive(spade.Behaviour.PeriodicBehaviour):
        # this behaviour is in charge of waiting for messages   
        def __init__(self, b):
            self.b = b
            self.c = [1,0,0]
            self.d = None
            self.msg = [1,0,0]
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.2)
            print "1"
            self.e1 = 0
        
        def onStart(self):
            print "listening for client"           
        def _onTick(self):
                 
            self.msg = self._receive(False, None, None)
            n = self.b.n                        
            #check wether the message arrived

            if self.msg:                
                print "%d got a message" %(n)
                self.c = self.msg.getContent()

            self.e1 = self.e1 +1 
        #def onEnd(self):
            #print "server closed"


    # agentid: the id of the agent sending message.  agserverid: the agentid recieving the message. agsdmessage: the sending message
    class Send(spade.Behaviour.PeriodicBehaviour):     
        # this behaviour is in charge of sending a messages to the receiver

        def __init__(self, b):    #serverid, sdmessage
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.1)   # achieve the upper level class in the son class. with this sentence, the variable and method can be used in this clss
            self.b = b
            self.i = 0
            self.sdmessage = [0,0,0]
            self.e2 = 0
        def _onTick(self):
              
          

            neighb = self.b.transfer_neighb() # network metrix
            p = self.b.transfer_command()  # the power request 

            today = datetime.datetime.now()
            self.b.time = time.mktime(today.timetuple()) #- self.b.xaxe 

            cap = self.b.transfer_powercap()
            mareq = self.b.transfer_macommand() 

            n = self.b.n 
            stru = self.b.transfer_stuc()  #system structure
            reac = self.b.transfer_reac()  #system line reactance array

            while ( neighb[self.i] == 0 and self.i != (len(neighb)-1)):                 
                self.i = self.i+1
            
            if neighb[self.i] == 1: 
                h = self.i+1    #print [2, content[0], content[1]] 
                serverid = "agent" + str(h) + "@127.0.0.1"      # str(2)                 
                self.sdmessage = [n,p,cap,mareq[self.i],self.b.transfer_control_timestamp()]   
                
                num = 20    
                #First, form the receiver AID
                addr = "xmpp://" + serverid
                receiver = spade.AID.aid(name = serverid, addresses = [addr])  #"xmpp://agent_server@127.0.0.1"
                
                #second, build the message
                self.msg = spade.ACLMessage.ACLMessage()    #initiate the message
                self.msg.setPerformative("inform")          #set the "information" FIPA performative
                self.msg.setOntology("myOntology")          #set the ontology of the message content
                self.msg.setLanguage("English")             #set the language of teh message content
                self.msg.addReceiver(receiver)              #add the message receiver

                #this content is consisted of the power exchanging between micro grid and network, the micro grid capacity
                self.msg.setContent(self.sdmessage)          #set the message content: str(client.sdmessage)
                
                #third, send the message with the "send" method of the agent
                self.myAgent.send(self.msg)
         
            else:
                pass

            if self.i < len(neighb)-1:
                self.i = self.i + 1
            else:
                self.i = 0


            self.e2 = self.e2 +1
  
    class Send_pf(spade.Behaviour.PeriodicBehaviour):     
        # this behaviour is in charge of sending a messages to the receiver

        def __init__(self, b):    
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.2)   
            self.b = b
            self.i = 0
            self.sdmessage = [0,0,0]
            self.e3 = 0
        def _onTick(self): 
   	
            neighb = self.b.transfer_neighb() # network metrix	                     
            n = self.b.n 

            while ( neighb[self.i] == 0 and self.i != (len(neighb)-1)):
                self.i = self.i+1

            if neighb[self.i] == 1: 
                h = self.i+1    
                serverid = "line" + str(min(h,n)) + "_" + str(max(h,n)) + "@127.0.0.1"                    
                self.sdmessage = [1,n,self.b.transfer_control_out(),self.b.transfer_control_timestamp(),0]       
          
                num = 20    
                #First, form the receiver AID
                addr = "xmpp://" + serverid
                receiver = spade.AID.aid(name = serverid, addresses = [addr])  #"xmpp://agent_server@127.0.0.1"
                #print addr
                #second, build the message
                self.msg = spade.ACLMessage.ACLMessage()    #initiate the message
                self.msg.setPerformative("inform")          #set the "information" FIPA performative
                self.msg.setOntology("myOntology")          #set the ontology of the message content
                self.msg.setLanguage("English")             #set the language of teh message content
                self.msg.addReceiver(receiver)              #add the message receiver

                #this content is consisted of the power exchanging between micro grid and network, the micro grid capacity
                self.msg.setContent(self.sdmessage)          #set the message content: str(client.sdmessage)
                
                #third, send the message with the "send" method of the agent
                self.myAgent.send(self.msg)
            else:
                pass

            if self.i < len(neighb)-1:
                self.i = self.i + 1
            else:
                self.i = 0


            self.e3 = self.e3 +1	                
            
    class store(spade.Behaviour.PeriodicBehaviour):
        def __init__(self, b):  
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.4)
            number = 13
            self.b = b
            self.dict_record = {}
            self.dict_max = {"version": 1}
            self.dict_max = {"version": 1}
            self.dict_cap = {"version": 1}
            self.dict_time = 0
            self.w = 0
            self.req = np.zeros(number)
            for i in range(number):
                self.dict_max[str(i+1)] = np.zeros(number)
                self.dict_cap[str(i+1)] = np.zeros(2)
            self.e4 = 0
                
        def _onTick(self):
            number = 13
            n = self.b.n               
            mes = self.b.transfer_Receive()

            for i in range(13):                           
                self.dict_max[str(i)][i] = 0  #for coordination with the forward programme
             
            self.dict_cap[str(1)][0] = 0  # dic key starts from "1"
            self.dict_cap[str(1)][1] = 0
            self.req[1-1] = 0   # array head starts from 0                
            self.dict_time = 0           
            self.dict_record[str(2)] = {str(1): 0}


            self.e4 = self.e4 +1            

    class importion(spade.Behaviour.PeriodicBehaviour):
        # 100 secondes once
        def __init__(self, b):
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.4)
            number = 13
            self.net_con = {"version":'1'}     #initialize the data storage dictionary: network
            self.mg_internal = {"version":'1'}     #initialize the data storage dictionary: internal micro grid
            self.neighb = np.zeros(number)
            self.mode = np.zeros(number)
            self.load = 0
            self.b = b
            self.timerow = 0
            self.con_struc = 0
            self.con_susce = 0
            self.con_reac = 0 
            self.stru_array = np.zeros([number,number])
            self.reac_array = np.zeros([number,number])
            self.load_dataf = 0
            self.mg_wind = {"version":'1'}
            self.wea_data = [0,0,0,0,0]
            self.e5 = 0
        def onStart(self):
            addr = os.path.split(os.path.realpath(__file__))[0]
            number = 13
            n = self.b.n
            print "start importing"
            self.i = 600    #720
            self.con_struc = pd.read_csv(join(addr,'con_structure.csv'))  # import the connection line
            self.con_susce = pd.read_csv(join(addr,'con_susceptance.csv'))   # import the line susceptance£¬ when i == j: b[i][j] = 0
            self.con_reac = pd.read_csv(join(addr,'con_reactance.csv'))   # import the line reactance£¬ when i == j: x[i][j] = 0
            self.load_dataf = pd.read_csv(join(addr,'load1.csv'))
            # array for the system connection and line reactance(when i=j, x[i][i] = -sum(x[i])) 
            for i in range(number):
                for j in range (number):
                    self.stru_array[i][j]=self.con_struc["mg"+str(i+1)][j]
                    self.reac_array[i][j]=self.con_reac["mg"+str(i+1)][j]
                self.reac_array[i][i] = -sum(self.reac_array[i])                          
            # name of the information files for micro grids, standardizing the name of the file is helpful for importing the batch in one shot
            # file name
            fname_struc = "structure_mg" + str(n) + ".csv"             
            fname_para = "parameter_mg" + str(n) + ".csv"
            fname_weth = "weather" + str(n) + ".csv" 
            # the contents of each micro grid
            self.mg_internal["structure_mg"] = pd.read_csv(join(addr,fname_struc))  #mkdir()
            # the facilities' basic parameters, which determines the character of the facilities. they will only depends on the production of the facilities
            self.mg_internal["parameter_mg"] = pd.read_csv(join(addr,fname_para))   #mkdir()
            # the wether in corresponding area
            self.mg_internal["weather"] = pd.read_csv(join(addr,fname_weth))
            self.mg_wind["total"] = pd.read_csv(join(addr,'weather_wind2.csv'))
            # establish a dictionary for the data storage            
            self.net_con["con_struct"] = self.con_struc     #save the connection line into dictionary
            self.net_con["con_suscep"] = self.con_susce      #save the line susception into dictionary
            self.mode = self.con_struc["mode"]

        def _onTick(self):
            
            address = 1    #address is the storage address for the data file, it can be used in the future as the address input 

            #import the data for individual micro grid : each micro grid has their own parameter, so the information should be imported one by one 
            
            n = self.b.n
            
            net_structrue = self.net_con["con_struct"]    # transfer the data dic into a simpler dic
            mg_name = "mg" + str(n)   # the index name of the row in .csv file
            load_name = "load" + str(n)
            self.neighb = net_structrue[mg_name]    # select the corresponding row for the mg. it shows the neighbors.

            #import load data    
            load_data = self.load_dataf[load_name]           

            #print self.mg_internal
            
            weather = self.mg_internal["weather"]   

            weather2 = self.mg_wind["total"]
            if n == 3:    
                wind_speed_data1 = weather2['windSpeed'] # wind speed 
                wind_degree_data1 = weather2['windDir'] # wind degree
                wind_speed_data2 = weather2['windGust'] # windgust speed 
                wind_degree_data2 = weather2['windGustDir'] # windgust degree
            else:
                wind_speed_data1 = weather['windSpeed'] # wind speed 
                wind_degree_data1 = weather['windDir'] # wind degree
                wind_speed_data2 = weather['windGust'] # windgust speed 
                wind_degree_data2 = weather['windGustDir'] # windgust degree                
            rad_data = weather['radiation'] # irradiance 

            #self.b.xaxe = weather['dateTime']

            #td3 = np.array(timerow2[:-1], dtype=np.datetime64)
            #print "222222222222222222222222222222: %r" %(td3)
            a = 0
            
            #while a<50: 
                #time.sleep(2)
            self.wea_data = [wind_speed_data1[self.i], wind_speed_data2[self.i], wind_degree_data1[self.i], wind_degree_data2[self.i], rad_data[self.i]]
            self.load = load_data[self.i]
            self.i = self.i +1
            if self.i == 1440:
                self.i = 0
            else:
                pass
            self.e5 = self.e5 +1            

    class batteryModel(spade.Behaviour.PeriodicBehaviour):

        def __init__(self, b):  #Capa, maxChargePower, maxDischargePower, initSoc, dt, battPower
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.2) 
            self.b = b
            self.out_mark = 0
            self.absorb_mark = 0
            #img = self.b.transfer_mg()
            #para = img["parameter_mg"]            
            self.initSoc = 20
            self.soc = self.initSoc   #change mark
            self.out_mark = 0
            #print "soc:", self.soc            
            self.e6 = 0
        def _onStart(self):
            print "battery start"

        def _onTick(self):     
            self.e6 = self.e6 +1

    class wdModel(spade.Behaviour.PeriodicBehaviour):
    #fac: efficiency factor   R:radius
        def __init__(self,b):    #fac, R, velocity, degree
            self.b = b
            self.output = 0
            self.outputp = 0    #the output of previous recycle
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.4)             
            self.e7 = 0
        def _onStart(self):
            print "windturbine start"

        def _onTick(self):

            self.e7 = self.e7 +1                                    
            
    class pvModel(spade.Behaviour.PeriodicBehaviour):
    # eta: factor  S:area
        def __init__(self, b):    #eta, S, irradiance
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.4)
            self.b = b
            self.output = 0
            self.e8 = 0
        def _onTick(self):

            self.output = self.eta*self.S*self.irradiance

            self.e8 = self.e8 +1
    class fuelcellModel(spade.Behaviour.PeriodicBehaviour):        
        def __init__(self, b):      #number_of_cells_in_series, operating_temperature, command_current
            spade.Behaviour.PeriodicBehaviour.__init__(self,600)
            self.b = b
            self.Pa_O2 = 2026
            self.Pa_H2 = 1013
            self.parametric_coe1 = -1.053
            self.parametric_coe3 = 7.6*10**(-5)
            self.parametric_coe4 = 1.93*10**(-4)
            self.A = 1.2
            self.B = 0.016
            self.l = 0.0178
            self.concentration_O2 = 2.119 * 10**(-3)
            self.concentration_H2 = 6.54 * 10**(-5)
            self.current_den = 0.0496
            self.resistance_connection = 0.0003
            self.factor = 16
            self.power = 0
            self.current_shortcircuit = 0.745
            self.e9 = 0
        def _onStart(self):
            print "fuelcell start"

        def _onTick(self):

            self.e9 = self.e9 +1     

    class microgrid(spade.Behaviour.PeriodicBehaviour):
        def __init__(self,b):
            spade.Behaviour.PeriodicBehaviour.__init__(self, 0.4)
            self.b = b
            print "5"
            self.bat = [0,0]
            self.powercap = [0,0]
            self.P_max = 0
            self.P_min = 0
            self.command = 0
            self.P_sum = 0
            self.batcommand = 0
            self.wind2_out = 0
            self.fc_out = 0
            self.pv2_out = 0
            self.mt_out = 0 
            self.mt_pmax = 5000
            self.e10 = 0    
        def _onTick(self):

            self.wind2_out = self.b.transfer_wdModel() 
            self.fc_out = self.b.transfer_fuelcellModel()                 
            self.pv2_out = self.b.transfer_pvModel()
            self.mt_out = 0
            load = self.b.transfer_load()                            
            self.P_sum  = self.wind2_out  + self.pv2_out  + self.fc_out  - load              
            self.command = 0
            self.powercap = [-10, 10]
            self.e10 = self.e10 +1

            
    class control(spade.Behaviour.PeriodicBehaviour):

        def __init__(self,b):
            spade.Behaviour.PeriodicBehaviour.__init__(self, 0.4)
            number = 13
            self.b = b 
            self.macommand = np.zeros(number)
            self.e = 0
            self.isolation = 0
            self.iteropt = np.zeros(number)
            self.k = 0
            self.B = np.zeros([number,number])
            self.control_out = 0
            self.time_stamp = 0
            self.e11 = 0
        def _onTick(self):       #process    

            today = datetime.datetime.now()
            self.timestamp = time.mktime(today.timetuple()) 
            self.b.powerout = self.b.transfer_command()
            self.control_out = self.b.transfer_command()
            self.isolation = 0
            for i in range(number):                       
                self.macommand[i] = 1              

            self.e11 = self.e11 +1


    class showout(spade.Behaviour.PeriodicBehaviour):
        def __init__(self,b):
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.4)
            self.b = b
            self.h = 0
            self.e12 = 0
        def _onTick(self):

            self.e12 = self.e12 +1
            self.b.e1 = np.concatenate((self.b.e1, [self.b.transfer_e1()]))
            self.b.e2 = np.concatenate((self.b.e2, [self.b.transfer_e2()]))
            self.b.e3 = np.concatenate((self.b.e3, [self.b.transfer_e3()]))
            self.b.e4 = np.concatenate((self.b.e4, [self.b.transfer_e4()]))
            self.b.e5 = np.concatenate((self.b.e5, [self.b.transfer_e5()]))
            self.b.e6 = np.concatenate((self.b.e6, [self.b.transfer_e6()]))
            self.b.e7 = np.concatenate((self.b.e7, [self.b.transfer_e7()]))
            self.b.e8 = np.concatenate((self.b.e8, [self.b.transfer_e8()]))            
            self.b.e9 = np.concatenate((self.b.e9, [self.b.transfer_e9()]))
            self.b.e10 = np.concatenate((self.b.e10, [self.b.transfer_e10()]))
            self.b.e11 = np.concatenate((self.b.e11, [self.b.transfer_e11()]))
            self.b.e12 = np.concatenate((self.b.e12, [self.e12]))
            #print "33333666666666666666666666666: "#%r %(self.b.trload)
            #print "33333666666666666666666666666: %r %r" %(self.b.load, self.b.time)           
            #print "666666666666666666666666: %r %r" %(self.b.test, today)          

    def transfer_net(self):
        #print self.thr_importion.net_con
        return self.thr_importion.net_con
    def transfer_neighb(self):
        return self.thr_importion.neighb
    def transfer_mode(self):
        return self.thr_importion.mode   
    def transfer_batcommand(self):
        return self.thr_microgrid.batcommand   #20
    def transfer_mg(self):
        return self.thr_importion.mg_internal
    def transfer_weather(self):
        return self.thr_importion.wea_data
    def transfer_load(self):
        return self.thr_importion.load
    def transfer_stuc(self):
        return self.thr_importion.stru_array
    def transfer_reac(self):
        return self.thr_importion.reac_array

    def transfer_soc(self):
        #print "battery:", self.thr_batteryModel.battery
        return self.thr_batteryModel.soc
    def transfer_out_mark(self):
        #print "battery:", self.thr_batteryModel.battery
        return self.thr_batteryModel.out_mark
    def transfer_wdModel(self):
        return self.thr_wdModel.output
    def transfer_fuelcellModel(self):
        return self.thr_fuelcellModel.power
    def transfer_pvModel(self):
        return self.thr_pvModel.output

    def transfer_Receive(self):
        #print "receive11111:", self.thr_Receive.c
        return self.thr_Receive.c   #[1,1,10]
    def transfer_powercap(self):
        return self.thr_microgrid.powercap
    def transfer_powersum(self):
        return self.thr_microgrid.P_sum
    def transfer_dict_max(self):
        return self.thr_store.dict_max    # get from neighbor agent
    def transfer_dict_cap(self):
        return self.thr_store.dict_cap    # get from neighbor agent
    def transfer_req(self):
        return self.thr_store.req    # get from control
    def transfer_dict_time(self):
        return self.thr_store.dict_time    # get from control        
    def transfer_command(self):
        return self.thr_microgrid.command    # get from control    

    def transfer_macommand(self):
        return self.thr_control.macommand    # get from control

    def transfer_iso(self):
        return self.thr_control.isolation
    def transfer_B(self):
        return self.thr_control.B        
    def transfer_control_out(self):
        return self.thr_control.control_out
    def transfer_control_timestamp(self):
        return self.thr_control.timestamp        

    def transfer_wind(self):
        return self.thr_microgrid.wind2_out    
    def transfer_fc(self):
        return self.thr_microgrid.fc_out
    def transfer_pv(self):
        return self.thr_microgrid.pv2_out
    def transfer_mt(self):
        return self.thr_microgrid.mt_out

    # interupt detection
    def transfer_e1(self):
        return self.thr_Receive.e1    
    def transfer_e2(self):
        return self.thr_Send.e2
    def transfer_e3(self):
        return self.thr_Send_pf.e3
    def transfer_e4(self):
        return self.thr_store.e4    
    def transfer_e5(self):
        return self.thr_importion.e5
    def transfer_e6(self):
        return self.thr_batteryModel.e6
    def transfer_e7(self):
        return self.thr_wdModel.e7    
    def transfer_e8(self):
        return self.thr_pvModel.e8
    def transfer_e9(self):
        return self.thr_fuelcellModel.e9
    def transfer_e10(self):
        return self.thr_microgrid.e10          
    def transfer_e11(self):
        return self.thr_control.e11 
     
    # initialize the agent message recieving
    def _setup(self):
        
        print "Server starting..."

        self.thr_Send = self.Send(self)
        self.thr_Send_pf = self.Send_pf(self)
        self.thr_Receive = self.Receive(self)
        self.thr_store = self.store(self)
        self.thr_importion = self.importion(self)
        self.thr_batteryModel = self.batteryModel(self)
        self.thr_wdModel = self.wdModel(self)
        self.thr_pvModel = self.pvModel(self)
        self.thr_fuelcellModel = self.fuelcellModel(self)
        self.thr_microgrid = self.microgrid(self)
        self.thr_control = self.control(self)
        self.thr_showout = self.showout(self)
        #self.thr_consensus = self.conalg(self)
              
        self.addBehaviour(self.thr_importion, None)
        self.addBehaviour(self.thr_Send, None)
        self.addBehaviour(self.thr_Send_pf, None)
        self.addBehaviour(self.thr_store, None)  #
        self.addBehaviour(self.thr_batteryModel, None)
        self.addBehaviour(self.thr_wdModel, None)
        self.addBehaviour(self.thr_pvModel, None)
        self.addBehaviour(self.thr_fuelcellModel, None)
        self.addBehaviour(self.thr_microgrid, None)
        self.addBehaviour(self.thr_control, None)
        self.addBehaviour(self.thr_showout, None)
        #self.addBehaviour(self.thr_consensus,None)        

        self.setDefaultBehaviour(self.thr_Receive)


        #time.sleep(10)
        #print b.msg

        #print self.b._process()

    def tearDown(self):
        self.stop()


class line(spade.Agent.Agent):
    def __init__(self, agentid, n1, n2):        
        
        self.agentid = agentid
        self.n1 = n1
        self.n2 = n2
        self.n1_mark = 0
        self.n2_mark = 0
        self.line_mark = 0
        self.send_mark = 0
        self.isoline_mark = 0
        self.in_reac = 0
        self.overflow = [0,0,0]
        self.powersum = [] # sum of power flow on the line
        self.powerout = 0
        number = 13
        self.request = [0]
        self.com12 = [0]
        self.com15 = [0]              
        spade.Agent.Agent.__init__(self, self.agentid, "secret")  
        con_struc = pd.read_csv('E:\parameter_test13\con_structure.csv')
        self.n1_nei = sum(con_struc["mg"+str(self.n1)])
        self.n2_nei = sum(con_struc["mg"+str(self.n2)])  
        self.e1 = [0]
        self.e2 = [0]    
        self.e3 = [0]
        self.e4 = [0] 
        self.e5 = [0]

    class Receive(spade.Behaviour.PeriodicBehaviour):   
        def __init__(self, b):
            self.b = b
            self.c = [1,0,0]
            self.d = None
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.2) 
            self.e1 = 0
        
            #print "1"
        # this behaviour is in charge of waiting for messages
        def onStart(self):
            print "listening for client"           
        def _onTick(self):

            self.msg = None
            self.msg = self._receive(False, None, None) #True, 0.2    False, None, None

            n = str(self.b.n1)+"_"+str(self.b.n2) 
            
            #check wether the message arrived
            if self.msg:                
                print "%r got a message" %(n)
                #if self.b.n1 == 3 and self.b.n2 == 4:
                    #print "the message is %r %r" %(self._presenceHandlers, self.exitCode())

                self.c = self.msg.getContent()
             
            self.e1 = self.e1+1 

        def onEnd(self):
            print "server closed"

    # agentid: the id of the agent sending message.  agserverid: the agentid recieving the message. agsdmessage: the sending message
    class Send(spade.Behaviour.PeriodicBehaviour):     
        # the request mg is n1 mg, the power flow is sending to lines connecting with n2 mg

        def __init__(self, b):    #serverid, sdmessage
            # achieve the upper level class in the son class. with this sentence, the variable and method can be used in this class
            number = 13
            self.b = b
            self.i = 0
            self.sdmessage = [0,0,0]
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.2) 
            self.e2 = 0  
        def _onTick(self):
           
            neighb = self.b.transfer_stru()
            rec = self.b.transfer_rec()
            pf = self.b.transfer_pf()
            serverid = "line" +"@127.0.0.1"


            while ((neighb[self.b.n2-1][self.i] == 0 or self.i == self.b.n1-1) and self.i != (len(neighb[self.b.n2-1])-1)):    
                self.i = self.i+1 

            if neighb[self.b.n2-1][self.i] != 0: 
                h = self.i+1

                serverid = "line" + str(min(self.b.n2,h)) + "_" + str(max(self.b.n2,h)) +"@127.0.0.1"

                # message = [0.signature of message, 1. n1, 2. n2, 3. time stamp, 4. power flow, 5. spread step, 6. initiator mg]                        
                self.sdmessage = [2,self.b.n1,self.b.n2,rec[1], pf,rec[2]+1,rec[3]]
                #First, form the receiver AID
                addr = "xmpp://" + serverid
                receiver = spade.AID.aid(name = serverid, addresses = [addr])  #"xmpp://agent_server@127.0.0.1"
                #print addr
                #second, build the message

                self.msg = spade.ACLMessage.ACLMessage()    #initiate the message
                self.msg.setPerformative("inform")          #set the "information" FIPA performative
                self.msg.setOntology("myOntology")          #set the ontology of the message content
                self.msg.setLanguage("English")             #set the language of the message content
                self.msg.addReceiver(receiver)              #add the message receiver

                #this content is consisted of the power exchanging between micro grid and network, the micro grid capacity
                self.msg.setContent(self.sdmessage)          #set the message content: str(client.sdmessage)
                
                #third, send the message with the "send" method of the agent
                self.myAgent.send(self.msg)

            else:
                pass



            if self.i < len(neighb[self.b.n2-1])-1:
                self.i = self.i + 1
            else:
                self.i = 0


            self.e2 = self.e2+1 
         

    class Send_error(spade.Behaviour.PeriodicBehaviour):     
        # the request mg is n1 mg, the power flow is sending to lines connecting with n2 mg

        def __init__(self, b):    #serverid, sdmessage
            # achieve the upper level class in the son class. with this sentence, the variable and method can be used in this class
            number = 13
            self.b = b
            self.i = 0
            self.sdmessage = [0,0,0]
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.1) 
            self.e3 = 0  
        def _onTick(self):

            self.e3 = self.e3+1 

    class store(spade.Behaviour.PeriodicBehaviour):
        def __init__(self, b):  #Capa, maxChargePower, maxDischargePower, initSoc, dt, battPower
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.2)
            number = 13
            self.b = b
            self.rec = [1,2,2,2]
            self.dict_line = {"version": 1}
            self.last_stream = np.zeros(3)
            #current_time = datetime.datetime.now()
            #self.lat_time = time.mktime(current_time.timetuple())
            self.pf_line = 0  
            self.sum = 0 
            self.sum1 = 0	        
            self.limitation = 3  
            self.e4 = 0       
        def _onTick(self):
            neighb = self.b.transfer_stru()
            number = 13                      
            mes = self.b.transfer_Receive()
            reac = self.b.transfer_reac()
            capacity = self.b.transfer_capa()

            now_time = datetime.datetime.now()
            now = time.mktime(now_time.timetuple()) 

            self.e4 = self.e4+1 

    class importion(spade.Behaviour.OneShotBehaviour):
        # 100 secondes once
        def __init__(self, b):
            spade.Behaviour.OneShotBehaviour.__init__(self)
            number = 13
            self.b = b
            self.stru_array = np.zeros([number,number])
            self.reac_array = np.zeros([number,number])
            self.susc_array = np.zeros([number,number])
            self.capa_array = np.zeros([number,number])
        def _process(self):
            addr = os.path.split(os.path.realpath(__file__))[0]
            number = 13            
            print "start importing"
            con_struc = pd.read_csv(join(addr,'con_structure.csv'))   # import the connection line
            con_susce = pd.read_csv(join(addr,'con_susceptance.csv'))   # import the line susceptance£¬ when i == j: b[i][j] = 0
            con_reac = pd.read_csv(join(addr,'con_reactance.csv'))   # import the line reactance£¬ when i == j: x[i][j] = 0
            con_capa = pd.read_csv(join(addr,'con_capacity.csv'))   # import the line capacity
            # array for the system connection and line reactance(when i=j, x[i][i] = -sum(x[i]), b[i][j] = 0) 
            for i in range(number):
                for j in range (number):
                    self.stru_array[i][j]=con_struc["mg"+str(i+1)][j]
                    self.reac_array[i][j]=con_reac["mg"+str(i+1)][j]
                    self.susc_array[i][j]=con_susce["mg"+str(i+1)][j]
                    self.capa_array[i][j]=con_capa["mg"+str(i+1)][j]
                self.reac_array[i][i] = -sum(self.reac_array[i])

    class showout(spade.Behaviour.PeriodicBehaviour):
        def __init__(self,b):
            spade.Behaviour.PeriodicBehaviour.__init__(self,0.4)
            self.b = b
            self.h = 0
            self.e5 = 0
        def _onTick(self):

            #today = datetime.datetime.now()
            #totime = time.mktime(today.timetuple()) - self.b.xaxe            
            #self.b.time = np.concatenate((self.b.time, [self.b.xaxe]))    #totime

            #print "111166666666666666666666666: %r" %(self.b.time)
            #e = self.b.transfer_iter_maxp()
            #self.b.test = np.concatenate((self.b.test, [e[1]]))          # test the consensus thorem, if there is consumption of other micro grid
            #self.b.test_consen = np.concatenate((self.b.test_consen, [e[0]]))

            #if self.b.n1 == 3 and self.b.n2 == 4:
                #print "777777777777777777777777777777777777777777777777777store: %r" %(self.b.transfer_powersum())        
            self.b.powersum = np.concatenate((self.b.powersum, [self.b.transfer_powersum()]))
            
            self.e5 = self.e5+1 
            self.b.e1 = np.concatenate((self.b.e1, [self.b.transfer_e1()]))
            self.b.e2 = np.concatenate((self.b.e2, [self.b.transfer_e2()]))
            self.b.e3 = np.concatenate((self.b.e3, [self.b.transfer_e3()]))
            self.b.e4 = np.concatenate((self.b.e4, [self.b.transfer_e4()]))
            self.b.e5 = np.concatenate((self.b.e5, [self.e5]))

    def transfer_reac(self):
        #print self.thr_importion.net_con
        return self.thr_importion.reac_array
    def transfer_stru(self):
        return self.thr_importion.stru_array
    def transfer_susc(self):
        return self.thr_importion.susc_array  
    def transfer_capa(self):
        return self.thr_importion.capa_array  

    def transfer_Receive(self):
        #print "receive11111:", self.thr_Receive.c
        return self.thr_Receive.c   #[1,1,10]

    def transfer_rec(self):
        return self.thr_store.rec    
    def transfer_pf(self):
        return self.thr_store.pf_line   
    def transfer_powersum(self):
        return self.thr_store.sum1  

    # interuption detection
    def transfer_e1(self):
        return self.thr_Receive.e1    
    def transfer_e2(self):
        return self.thr_Send.e2   
    def transfer_e3(self):
        return self.thr_Send_error.e3 
    def transfer_e4(self):
        return self.thr_store.e4                      

# initialize the agent message recieving
    def _setup(self):
        
        print "Server starting..."
        self.thr_importion = self.importion(self)
        self.thr_Send = self.Send(self)             
        self.thr_Receive = self.Receive(self)
        self.thr_store = self.store(self)
        self.thr_Send_error = self.Send_error(self)
        self.thr_showout = self.showout(self)

        #self.thr_showout = self.showout(self)
              
        self.addBehaviour(self.thr_importion, None)
        self.addBehaviour(self.thr_Send, None)
        self.addBehaviour(self.thr_store, None)  #
        self.addBehaviour(self.thr_Send_error, None)
        self.addBehaviour(self.thr_showout, None)

        #self.addBehaviour(self.thr_showout, None)

        self.setDefaultBehaviour(self.thr_Receive)


        #time.sleep(10)
        #print b.msg

        #print self.b._process()

    def tearDown(self):
        self.stop()



if __name__ == "__main__":



    num_MG = 13
    A = {"version": 1}
    B = {"version": 1}
    
    for i in range(num_MG):
        n = "agent" + str(i+1) +"@127.0.0.1"
        A[str(i)] = MyAgent(n,(i+1))   # A starts from 0 and ends at 11 
        A[str(i)].start()
    time.sleep(2)
    #k = MyPF("agent@127.0.0.1", A)
    #k.start()

    network_ary = np.zeros([num_MG, num_MG])
    # import the network structure
    network = pd.read_csv('E:\parameter_test13\con_structure.csv')
    for i in range(num_MG):
        for j in range(num_MG):            
            if network["mg" + str(i+1)][j] != 0 and (i+1) <= j:
                # start line agent
                B[str(i+1)+"_"+str(j+1)] = line("line" + str(i+1) + "_" + str(j+1) +"@127.0.0.1", (i+1), (j+1))
                B[str(i+1)+"_"+str(j+1)].start()
                logger = logging.getLogger('mylogger')  

    time.sleep(200) 

    for i in range(num_MG):            
        A[str(i)].tearDown() 

    for i in range(num_MG):
        for j in range(num_MG):            
            if network["mg" + str(i+1)][j] != 0 and (i+1) <= j:
                # start line agent
                B[str(i+1)+"_"+str(j+1)].tearDown()

    a1 = A["0"]
    a2 = A["1"]
    a3 = A["2"]
    a4 = A["3"]
    a5 = A["4"]
    weat = pd.read_csv('E:\parameter_test5\weather1.csv')
    td1 = weat['dateTime']
    timerow2 = pd.to_datetime(td1, unit='s')
    timerow = np.array(timerow2[:-1], dtype=np.datetime64)    

    n = min(len(a1.e1),len(timerow),len(a2.e1),len(a3.e1),len(a4.e1),len(a5.e1))
    m1 = np.array(a1.e1[:n])
    m2 = np.array(a2.e1[:n])
    m3 = np.array(a3.e1[:n])
    m4 = np.array(a4.e1[:n])   # np.array(a2.parray[:n])
    m5 = np.array(a5.e1[:n])

    t = np.array(timerow[:n])
    #print "66666666666666666666666666666666666666666666666666666666666666666666666666666666: %r %r" %(t, m2)


    fig = plt.figure(figsize=(10,10)) 
    ax = plt.subplot(111) 
    plt.title('Output of MG1 Internal Facilities') 
    MG1 = plt.plot_date(t, m1, fmt='b')  
    MG2 = plt.plot_date(t, m2, fmt='k:',linewidth=2) 
    MG3 = plt.plot_date(t, m3, fmt='r--',linewidth=2) 
    MG4 = plt.plot_date(t, m4, fmt='g-.')
    MG5 = plt.plot_date(t, m5, fmt='y')
    plt.xlabel('Time: /s') 
    plt.ylabel('Output: /W') 
    plt.legend(MG1 + MG2 + MG3 + MG4 + MG5 , ['1','2','3','4','5'],'best',numpoints=1) 
    plt.show()

    n1 = min(len(B["1_2"].e1),len(B["1_2"].e2),len(timerow),len(B["2_7"].e2),len(B["2_7"].e1))
    t2 = np.array(timerow[:n1])
    plot1 = plt.plot(t2,B["1_2"].e1[:n1],'r')
    plot2 = plt.plot(t2,B["1_2"].e2[:n1],'r--')
    plot3 = plt.plot(t2,B["2_7"].e2[:n1],'g--',linewidth=2)
    plot4 = plt.plot(t2,B["2_7"].e1[:n1],'g',linewidth=2)
    plt.title('Plot of Power Flow')
    plt.xlabel('time: /s')
    plt.ylabel('power: /w')
    plt.legend(plot1+plot2+plot3+plot4,('12_1','12_2','27_2','27_1'),'best',numpoints=1)
    plt.show()


