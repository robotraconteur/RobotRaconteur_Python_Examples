#Find iRobot Create Service Node using Robot Raconteur autodiscovery

from RobotRaconteur.Client import *
import time

def main():
    #Initialize Robot Raconteur
    RRN.UseNumPy=True

    #Wait for ten seconds for the node discovery to occur
    time.sleep(10)

    #Find the node
    res=RRN.FindServiceByType("experimental.create.Create",["rr+local","rr+tcp"])
    for r in res:
        print r.NodeName + " " + str(r.NodeID) + " " +  r.Name + " " + r.ConnectionURL[0]

    if (len(res)==0):
        print "No Create robot found!"
    else:
        #Connect and drive a bit, then disconnect
        c=RRN.ConnectService(res[0].ConnectionURL)
        c.Drive(200,5000)
        time.sleep(1)
        c.Drive(0,0)
        RRN.DisconnectService(c)

if __name__ == '__main__':
    main()
