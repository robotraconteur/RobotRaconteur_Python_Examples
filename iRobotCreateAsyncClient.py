from RobotRaconteur.Client import *
import time
import thread
import numpy
import threading
import sys

ev=threading.Event()
err=None

def client_handler(e):
    global err
    err=e
    ev.set()

class AsyncCreateClient(object):
    def __init__(self,handler):
        self._handler=handler
        self._c=None
    def start(self):
        try:
            url='rr+tcp://localhost:2354?service=Create'
            if (len(sys.argv)>=2):
                url=sys.argv[1]
            RRN.AsyncConnectService(url,None,None,None,self.handler1,5)
        except Exception as e:
            self._handler(e)

    def handler1(self,c,err):
        if (err is not None):
            self._handler(err)
            return
        try:
            c.async_get_Bumpers(self.handler2,0.5)
            self.c=c
        except Exception as e:
            self._handler(e)

    def handler2(self,bumpers,err):
        if (err is not None):
            self._handler(err)
            return
        try:
            self.c.async_set_Bumpers(10,self.handler3,0.5)
        except Exception as e:
            self._handler(e)

    def handler3(self,err):
        #In this case we expect an error because this is read only
        if (err is None):
            self._handler(Exception("Expected an error"))
            return
        try:
            RRN.AsyncDisconnectService(self.c,self.handler4)
        except Exception as e:
            self._handler(e)

    def handler4(self):
        self._handler(None)


def main():

    RRN.UseNumPy=True

    c=AsyncCreateClient(client_handler)
    c.start()

    ev.wait()

    if (err is None):
        print "No error occured!"
    else:
        print "Error occured: " + repr(err)

if __name__ == '__main__':
    main()
