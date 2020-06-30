#!/usr/bin/env python2.7

import sys
import rospy
from geometry_msgs.msg import Twist
import serial


'''
TODOS AS FUNÇÕES SERÃO ALTERADAS QUANDO ESTIVER 
FEITO A PARTE DE ENVIO E LEITURA PELA SERIAL.
'''

class getValue:
    def __init__(uart):
        self.serial = uart

    def readMotorAmps():
        #RECEBER OS DADOS E RETORNAR UM VETOR COM O VALOR DOS DOIS DADOS
        serial.write('?A'+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1].split(':')
        return receivedData 
        

    def readBatteryAmps():
        serial.write('?BA R:'+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1].split(':')
        return receivedData 


    def readEncoderCounteAbsolute(motorch)
        serial.write('?C ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData

    def readEncoderCountRelative(motorch):
        serial.write('?CR ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData

    def readClosedLoopError(motorch):
        serial.write('?E ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData    
    
    def readFeedback(motorch):
        serial.write('?F ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 

    def readMotorPowerOutputApplied(motorch):
        serial.write('?P ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 

    def readEncoderMotorSpeedinRPM(motorch):
        serial.write('?S ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 


    def readEncoderSpeedRelative(motorch):
        serial.write('?SR ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 

    def readTemperature(channel)
    ''' channel 1: MCU
        channel 2: channel1 side
        channel 3: channel2 side'''
        serial.write('?T ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 


     def readPositionRelativeTracking(motorch)
        serial.write('?TR ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 

        def readVolts():
    ''' 1 : Internal volts
        2 : Battery volts
        3 : 5V output'''  
        serial.write('?V'+'\r')
        serial.read_until('\r')
        receivedData = str(serial.serial.read_until('\r'))
        receivedData = receivedData.split('=')[1].split(':')
        return receivedData 

class setCommand:
    def __init__(uart):
        self.serial = uart

    def sendData(sendData)
        self.serial.write(sendData)
        receivedData = self.serial.read_until('\r')
        self.serial.read_until('\r')
        if (sendData == receivedData)
            return 1
        else:
            return -1

    def setAceleration(motorch, acel):
    #Acceleration value is in 0.1 * RPM per second.    
        command = '!AC ' + str(motorch) +' '+ str(acel)+'\r'
        return sendData(command)

    def setEncoderCounters(motorch, conter):
        command = '!C '+str(motorch)+' '+str(conter)+'\r'
        return sendData(command)
        
    def setDeceleration(motorch, deccel):
    #Decceleration value is in 0.1 * RPM per second    
        command = '!DC '+str(motorch)+' '+str(deccel)+'\r'
        return sendData(command)

    def goToSpeed(motorch, vel):
        command = '!G '+str(motorch)+' '+str(vel)+'\r'
        return sendData(command)

    def setMotorSpeed(motorch, speed):
        command = '!S '+ str(motorch)+' '+str(speed)+'\r'
        return sendData(command)


class hdc2450:
    """docstring for hdc245."""

    def __init__(self):
        SAFETYKEY = '321654987'
        baudRate  = 115200 
        serialPort = '/dev/serial0'
        uart = serial.Serial(self.serialPort, self.baudRate)
        self.getValue = getValue(uart)
        self.setCommand = setCommand(uart)
        
    def reset():
        command = '%RESET' + str(SAFETYKEY)
        uart.write(command)

    def emergencyStop():
        uart.write('!ES')


    def emergencyStopReleased():
        uart.write('!MG')