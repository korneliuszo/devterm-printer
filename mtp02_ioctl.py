import fcntl, struct

def sizeof(type): return struct.calcsize(type)
def _IOC(dir, type, nr, size):  return int((dir << _IOC_DIRSHIFT ) | (type << _IOC_TYPESHIFT ) |\
                                       (nr << _IOC_NRSHIFT ) | (size << _IOC_SIZESHIFT))
def _IO(type, nr):      return _IOC(_IOC_NONE,  type, nr, 0)
def _IOR(type,nr,size): return _IOC(_IOC_READ,  type, nr, sizeof(size))
def _IOW(type,nr,size): return _IOC(_IOC_WRITE, type, nr, sizeof(size))

_IOC_SIZEBITS   = 14
_IOC_SIZEMASK   = (1 << _IOC_SIZEBITS ) - 1
_IOC_NRSHIFT    = 0
_IOC_NRBITS     = 8
_IOC_TYPESHIFT  = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_TYPEBITS   = 8
_IOC_SIZESHIFT  = _IOC_TYPESHIFT + _IOC_TYPEBITS
IOCSIZE_MASK    = _IOC_SIZEMASK << _IOC_SIZESHIFT
IOCSIZE_SHIFT   = _IOC_SIZESHIFT

_IOC_WRITE      =  1
_IOC_READ       =  2
_IOC_INOUT      =  3

_IOC_DIRSHIFT   = _IOC_SIZESHIFT + _IOC_SIZEBITS
IOC_INOUT       = _IOC_INOUT << _IOC_DIRSHIFT
IOC_IN          = _IOC_WRITE << _IOC_DIRSHIFT
IOC_OUT         = _IOC_READ << _IOC_DIRSHIFT

_IOC_NONE       = 0

settings_keys = ['close_feed','line_feed','burn_time','burn_count','bytesatonce']
settings_struct = 'iiiii'

MTP02_FEED = _IO(100, 0)
MTP02_GET_SETTINGS = _IOR(100,  1, settings_struct)
MTP02_SET_SETTINGS = _IOW(100,  2, settings_struct)

def feed(dev, lines):
    fcntl.ioctl(dev, MTP02_FEED, lines)


def get_settings(dev):
    buff=bytearray(sizeof(settings_struct))
    fcntl.ioctl(dev, MTP02_GET_SETTINGS, buff)
    return dict(zip(settings_keys, struct.unpack(settings_struct,buff)))


def set_settings(dev,settings):
    buff = struct.pack(settings_struct,*[settings[v] for v in settings_keys])
    fcntl.ioctl(dev, MTP02_SET_SETTINGS, buff, False)








    