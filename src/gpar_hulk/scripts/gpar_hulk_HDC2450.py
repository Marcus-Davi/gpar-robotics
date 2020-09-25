#!/usr/bin/env python3

import sys
import serial


'''
TODOS AS FUNÇÕES SERÃO ALTERADAS QUANDO ESTIVER 
FEITO A PARTE DE ENVIO E LEITURA PELA SERIAL.
'''

'''
MOTOR 1 -> DIREITA
MOTOR 2-> ESQUERDA
'''

class GetValue:
    def __init__(self, uart):
        serial = uart

    def readMotorAmps(self):
        #RECEBER OS DADOS E RETORNAR UM VETOR COM O VALOR DOS DOIS DADOS
        serial.write('?A'+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1].split(':')
        return receivedData 
        

    def readBatteryAmps(self):
        serial.write('?BA R:'+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1].split(':')
        return receivedData 


    def readEncoderCounteAbsolute(self, motorch):
        serial.write('?C ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData

    def readEncoderCountRelative(self, motorch):
        serial.write('?CR ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData

    def readClosedLoopError(self, motorch):
        serial.write('?E ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData    
    
    def readFeedback(self, motorch):
        serial.write('?F ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 

    def readMotorPowerOutputApplied(self, motorch):
        serial.write('?P ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 

    def readEncoderMotorSpeedinRPM(self, motorch):
        serial.write('?S ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 


    def readEncoderSpeedRelative(self, motorch):
        serial.write('?SR ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 

    def readTemperature(self, channel):
        ''' 
        channel 1: MCU
        channel 2: channel1 side
        channel 3: channel2 side'''
        serial.write('?T ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 


    def readPositionRelativeTracking(self, motorch):
        serial.write('?TR ' + str(motorch)+'\r')
        serial.read_until('\r')
        receivedData = str(serial.serial.read_until('\r'))
        receivedData = receivedData.split('=')[1]
        return receivedData 

    def readVolts(self):
        ''' 
        1 : Internal volts
        2 : Battery volts
        3 : 5V output'''  
        serial.write('?V'+'\r')
        serial.read_until('\r')
        receivedData = str(serial.serial.read_until('\r'))
        receivedData = receivedData.split('=')[1].split(':')
        return receivedData 

class SetCommand:
    def __init__(self, uart):
        self.serial = uart

    def sendData(self, sendData):
        print(sendData.encode())
        self.serial.write(sendData.encode())
        self.serial.read_all()

    def setAceleration(self, motorch, acel):
    #Acceleration value is in 0.1 * RPM per second.    
        command = '!AC ' + str(motorch) +' '+ str(acel)+'\r'
        sendData(command)

    def setEncoderCounters(self, motorch, conter):
        command = '!C '+str(motorch)+' '+str(conter)+'\r'
        sendData(command)
        
    def setDeceleration(self, motorch, deccel):
    #Decceleration value is in 0.1 * RPM per second    
        command = '!DC '+str(motorch)+' '+str(deccel)+'\r'
        sendData(command)

    def goToSpeed(self, veld, vele):
        command = '!G 1 '+str(veld)+'\r'
        self.sendData(command)
        command ='!G 2 '+str(vele)+'\r'
        self.sendData(command)


    def setMotorSpeed(self, motorch, speed):
        command = '!S '+ str(motorch)+' '+str(speed)+'\r'
        sendData(command)


class hdc2450:
    """docstring for hdc245."""

    def __init__(self):
        SAFETYKEY = '321654987'
        baudRate  = 115200 
        serialPort = '/dev/ttyS0'
        self.uart = serial.Serial(serialPort, baudRate)
        self.getValue = GetValue(self.uart)
        self.setCommand = SetCommand(self.uart)
        
    def reset():
        command = '%RESET' + str(SAFETYKEY)
        uart.write(command)

    def emergencyStop():
        uart.write('!ES')


    def emergencyStopReleased():
        uart.write('!MG')