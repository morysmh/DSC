from logging import exception

def get_crc8(data):
    _polynome   = 0x7
    _startMask  = 0
    _endMask    = 0
    _crc        = 0
    _reverseIn  = False
    _reverseOut = False
    _started    = False
    e_count      = 0
    data = data + 'afb4'
    l = list(data)
    for i in l:
        _crc = _crc ^ ord(i)
        for l in range(0,8):
            if(_crc & (1<<7)):
                _crc = ((_crc<<1) & 0xFF)
                _crc = _crc ^ _polynome
            else:
                _crc = ((_crc << 1) & 0xFF)
    rv = _crc
    return ((_crc ^ _endMask) & 0xFF)


def get_crc16(data):
    _polynome   = 0x1021
    _startMask  = 0
    _endMask    = 0
    _crc        = 0
    _reverseIn  = False
    _reverseOut = False
    _started    = False
    _count      = 0
    _canYield   = True
    data = data + 'afb4'
    l = list(data)
    for i in l:
        _crc = _crc ^ ((ord(i) & 0xFF)<<8)
        for l in range(0,8):
            _crc = _crc & 0xFFFF
            if(_crc & (1<<15)):
                _crc = (_crc<<1)
                _crc = (_crc ^ _polynome) 
            else:
                _crc = (_crc << 1)
            _crc = _crc & 0xFFFF
    rv = _crc
    return ((_crc ^ _endMask) & 0xFFFF)


def test_crc():
    if get_crc8('98:CD:AC:22:F9:54') != 3:
        raise exception("Code not working")
    if get_crc8('98:CD:AC:22:E6:6E') != 238:
        raise exception("Code not working")
    if get_crc8('98:CD:AC:22:BB:0D') != 219:
        raise exception("Code not working")
    if get_crc8('98:CD:AC:22:AE:E1') != 4:
        raise exception("Code not working")
    if get_crc16('98:CD:AC:22:F9:54') != 58594:
        raise exception("CRC16 Code not working")
    if get_crc16('98:CD:AC:22:BB:0D') != 16425:
        raise exception("CRC16 Code not working")
    print("=========================")
    print("Every Thing seems to work")
    print("=========================")
    
#test_crc()

#print(get_crc16("(a14=2541:-1#"))
#print(get_crc16("my name is the best 12351234 01234098340158276983459"))
#print(get_crc16("a140=761"))