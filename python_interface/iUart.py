class iUart():
    '''
    Description:
    Communication class for Uartbase.py
    '''
    def __init__(self):
        pass
    
    def send(self):
        raise(NotImplementedError('Send not implemented'))
    
    def listen(self):
        raise(NotImplementedError('listen not implemented'))
    
    def Close(self):
        raise(NotImplementedError('close not implemented'))
    
    def Open(self):
        raise(NotImplementedError('open not implemented'))
        
    def ClearBuffer(self):
        raise(NotImplementedError('ClearBuffer not implemented'))